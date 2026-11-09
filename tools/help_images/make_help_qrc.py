# MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com>
# Writes resources/help/help.qrc from the guide's files on disk (the chapters and resources/help/images),
# and checks every picture a chapter uses is there.
#
#   python tools/help_images/make_help_qrc.py
import os
import re
import sys

root = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'resources', 'help')
root = os.path.normpath(root)
html = sorted(f for f in os.listdir(root) if f.endswith('.html'))
imgdir = os.path.join(root, 'images')
images = sorted(f for f in os.listdir(imgdir) if f.endswith('.png')) if os.path.isdir(imgdir) else []

used = set()
for f in html:
    for m in re.finditer(r'<img[^>]*src="images/([^"]+)"', open(os.path.join(root, f), encoding='utf-8').read()):
        used.add(m.group(1))
missing = sorted(used - set(images))
unused = sorted(set(images) - used)

lines = ['<RCC>', '  <qresource prefix="/help">']
lines += ['    <file>%s</file>' % f for f in html]
lines += ['    <file>images/%s</file>' % f for f in images if f in used]
lines += ['  </qresource>', '</RCC>', '']
open(os.path.join(root, 'help.qrc'), 'w', encoding='utf-8', newline='\n').write('\n'.join(lines))
print('help.qrc: %d chapters, %d pictures' % (len(html), len([f for f in images if f in used])))
if unused:
    print('not used by any chapter (left out): ' + ', '.join(unused))
if missing:
    print('MISSING pictures: ' + ', '.join(missing))
    sys.exit(1)
