#!/usr/bin/env python3
import argparse,time
from pathlib import Path
def main():
 p=argparse.ArgumentParser();p.add_argument('log');p.add_argument('--hz',type=float,default=20);a=p.parse_args()
 for l in Path(a.log).read_text().splitlines(): print(l,flush=True);time.sleep(1/a.hz)
if __name__=='__main__':main()
