#!/usr/bin/env python3
import json,sys
from pathlib import Path
p=Path(sys.argv[1]); d=json.loads(p.read_text()); Path(sys.argv[2] if len(sys.argv)>2 else 'vision_performance.md').write_text('# Vision performance\n\n```json\n'+json.dumps(d,indent=2)+'\n```\n')
