/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */

#include "FlashStore.h"

#include <sam.h>
#include <string.h>

namespace {

const uint32_t kMagic = 0x53504345u; /* 'ECPS' */
const uint16_t kVersion = 1;

struct Header {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint32_t length;
    uint32_t checksum;
};
static_assert(sizeof(Header) == 16, "header is 16 bytes");

uint32_t Fnv(const char *p, uint32_t n) {
    uint32_t h = 2166136261u;
    for (uint32_t i = 0; i < n; i++) {
        h ^= static_cast<uint8_t>(p[i]);
        h *= 16777619u;
    }
    return h;
}

const uint32_t kErrors = NVMCTRL_INTFLAG_ADDRE | NVMCTRL_INTFLAG_PROGE | NVMCTRL_INTFLAG_LOCKE | NVMCTRL_INTFLAG_NVME;

void (*g_yield)() = nullptr;

/* A block erase takes up to ~200 ms; give up after about a second (DWT cycle counter). */
bool WaitReady() {
    const uint32_t start = DWT->CYCCNT;
    while (DWT->CYCCNT - start < SystemCoreClock) {
        if (NVMCTRL->STATUS.bit.READY) {
            return true;
        }
        if (g_yield) {
            g_yield();
        }
    }
    return NVMCTRL->STATUS.bit.READY;
}

/* Run one command on addr; nullptr or the controller's complaint. */
const char *Command(uint32_t cmd, uint32_t addr) {
    if (!WaitReady()) return "flash controller busy";
    NVMCTRL->INTFLAG.reg = kErrors;  /* clear old error flags */
    NVMCTRL->ADDR.reg = addr;
    NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | cmd;
    if (!WaitReady()) return "flash command timed out";
    const uint32_t f = NVMCTRL->INTFLAG.reg;
    if (f & NVMCTRL_INTFLAG_LOCKE) return "flash region is locked";
    if (f & NVMCTRL_INTFLAG_ADDRE) return "flash address error";
    if (f & NVMCTRL_INTFLAG_PROGE) return "flash programming error";
    if (f & NVMCTRL_INTFLAG_NVME) return "flash controller error";
    return nullptr;
}

/* The CPU's cache (CMCC) is off while the controller erases and writes, and is invalidated before it
 * comes back on so it never serves the area's old contents. INVALL is only allowed with the cache
 * disabled (CSTS = 0); writing it with the cache on hung the board. */
class CacheOff {
public:
    CacheOff() : m_wasOn(CMCC->SR.bit.CSTS) {
        if (m_wasOn) {
            CMCC->CTRL.reg = 0;
            while (CMCC->SR.bit.CSTS) {
                continue;
            }
        }
    }
    ~CacheOff() {
        if (m_wasOn) {
            CMCC->MAINT0.reg = CMCC_MAINT0_INVALL;
            CMCC->CTRL.reg = CMCC_CTRL_CEN;
        }
    }

private:
    bool m_wasOn;
};

}  // namespace

void FlashStoreSetYield(void (*fn)()) {
    g_yield = fn;
}

const char *FlashStoreSave(const char *text, uint32_t length) {
    if (length > ECAM_STORE_TEXT_MAX) {
        return "profile too large for the flash store";
    }
    Header h;
    h.magic = kMagic;
    h.version = kVersion;
    h.reserved = 0;
    h.length = length;
    h.checksum = Fnv(text, length);

    const char *err = nullptr;
    {
    CacheOff cacheOff;
    const uint32_t savedMode = NVMCTRL->CTRLA.bit.WMODE;
    NVMCTRL->CTRLA.bit.WMODE = NVMCTRL_CTRLA_WMODE_MAN_Val;

    /* Erase the blocks the profile needs. */
    const uint32_t total = sizeof(Header) + length;
    for (uint32_t off = 0; off < total && !err; off += NVMCTRL_BLOCK_SIZE) {
        err = Command(NVMCTRL_CTRLB_CMD_EB, ECAM_STORE_ADDRESS + off);
    }

    /* Program page by page: fill the page buffer (32-bit writes to the page's address), then WP. */
    for (uint32_t off = 0; off < total && !err; off += NVMCTRL_PAGE_SIZE) {
        uint32_t page[NVMCTRL_PAGE_SIZE / 4];
        uint8_t *bytes = reinterpret_cast<uint8_t *>(page);
        memset(page, 0xFF, sizeof(page));
        for (uint32_t i = 0; i < NVMCTRL_PAGE_SIZE && off + i < total; i++) {
            const uint32_t at = off + i;
            bytes[i] = at < sizeof(Header) ? reinterpret_cast<const uint8_t *>(&h)[at]
                                           : static_cast<uint8_t>(text[at - sizeof(Header)]);
        }
        err = Command(NVMCTRL_CTRLB_CMD_PBC, ECAM_STORE_ADDRESS + off);
        if (err) break;
        volatile uint32_t *dst = reinterpret_cast<volatile uint32_t *>(ECAM_STORE_ADDRESS + off);
        for (uint32_t i = 0; i < NVMCTRL_PAGE_SIZE / 4; i++) {
            dst[i] = page[i];
        }
        err = Command(NVMCTRL_CTRLB_CMD_WP, ECAM_STORE_ADDRESS + off);
    }

    NVMCTRL->CTRLA.bit.WMODE = savedMode;
    }  /* cache back on, invalidated */
    if (err) {
        return err;
    }
    /* Read it back. */
    uint32_t n = 0;
    const char *stored = FlashStoreText(&n);
    if (!stored || n != length || memcmp(stored, text, length) != 0) {
        return "flash verify failed";
    }
    return nullptr;
}

const char *FlashStoreText(uint32_t *length) {
    const Header *h = reinterpret_cast<const Header *>(ECAM_STORE_ADDRESS);
    if (h->magic != kMagic || h->version != kVersion || h->length > ECAM_STORE_TEXT_MAX) {
        return nullptr;
    }
    const char *text = reinterpret_cast<const char *>(ECAM_STORE_ADDRESS + sizeof(Header));
    if (Fnv(text, h->length) != h->checksum) {
        return nullptr;
    }
    *length = h->length;
    return text;
}

const char *FlashStoreErase() {
    CacheOff cacheOff;
    const uint32_t savedMode = NVMCTRL->CTRLA.bit.WMODE;
    NVMCTRL->CTRLA.bit.WMODE = NVMCTRL_CTRLA_WMODE_MAN_Val;
    const char *err = Command(NVMCTRL_CTRLB_CMD_EB, ECAM_STORE_ADDRESS);
    NVMCTRL->CTRLA.bit.WMODE = savedMode;
    return err;
}
