/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#pragma once

// The user guide: chapters of HTML with pictures, kept in the resources (resources/help). A contents
// tree on the left (chapters and their sections), the page on the right, back / forward, find, and
// printing: the chapter, or the whole guide, on paper or as a PDF (light, ink-friendly styling; pictures
// scaled to the page). One window for the app; F1 / Help menu opens it, the scope opens its chapter.
//
// The pictures are made by tools/help_images (it drives the real windows and crops them), so they can be
// remade whenever the UI changes.

#include <QDialog>
#include <QSet>
#include <QString>
#include <QVector>

class QImage;
class QLineEdit;
class QTextDocument;
class QPrinter;
class QTextBrowser;
class QTreeWidget;
class QTreeWidgetItem;
class QUrl;

class HelpWindow : public QDialog {
    Q_OBJECT
public:
    struct Chapter {
        QString file;   // under :/help
        QString title;
    };
    static const QVector<Chapter> &Chapters();

    // Show the guide (one window for the app) at `page`, e.g. "12-scope.html" or "04-profile.html#knots";
    // empty: where it was, or the first chapter.
    static HelpWindow *Show(QWidget *parent, const QString &page = {});

    void open(const QString &page);
    QTextBrowser *browser() const { return m_browser; }
    QString currentPage() const;  // the chapter file shown
    QString contentsCurrent() const;  // the contents entry highlighted

    // The whole guide, or one chapter, as print-ready HTML (light styling, pictures sized for paper).
    static QString PrintHtml(const QString &chapterFile = {});
    // That HTML as a document ready to print (pictures attached, a page break before the headings in
    // `pageBreaks`, counted in document order); the caller owns it.
    static QTextDocument *PrintDocument(const QString &chapterFile = {}, QObject *parent = nullptr,
                                        const QSet<int> &pageBreaks = {});
    // Print the guide (or one chapter) to a printer already set up, or to a PDF file.
    static void PrintTo(QPrinter *printer, const QString &chapterFile = {});
    static bool SavePdf(const QString &path, QString *error, const QString &chapterFile = {});
    // One printed page (Letter, the print margins) as an image at `dpi`, and the page count: to check the
    // print layout without a printer.
    static QImage PreviewPage(int page, double dpi, int *pages, const QString &chapterFile = {});
    // The whole guide's cover page (printed and in the PDF before the first chapter), as a Letter page at dpi.
    static QImage PreviewCover(double dpi);
    // The whole guide's contents pages (after the cover): one as a Letter page at dpi, and how many there are.
    static QImage PreviewContents(int page, double dpi, int *pages);
    // A saved PDF's bookmarks in order, (level, title), level 1 a chapter; empty when it has none. For tests.
    static QVector<QPair<int, QString>> PdfOutlineForTest(const QString &path);
    // Headings left on another page than the start of what follows them (should be none).
    static int StrandedHeadings(const QString &chapterFile = {});

private:
    explicit HelpWindow(QWidget *parent);
    void BuildToc();
    void SyncToc(const QUrl &url);
    void Find(bool backwards);
    void Print(bool all);
    void ExportPdf();

    QTreeWidget *m_toc = nullptr;
    QTextBrowser *m_browser = nullptr;
    QLineEdit *m_find = nullptr;
};
