#!/usr/bin/env python3
import argparse,time
try: import serial
except ImportError: raise SystemExit('pyserial is required')
def main():
 p=argparse.ArgumentParser();p.add_argument('--port',default='/dev/dtask_openmv');p.add_argument('--seconds',type=float,default=10);a=p.parse_args();s=serial.Serial(a.port,115200,timeout=.1);n=0;t=time.monotonic()
 while time.monotonic()-t<a.seconds:
  if s.readline():n+=1
 print({'serial_lines_hz':n/a.seconds,'samples':n})
if __name__=='__main__':main()
