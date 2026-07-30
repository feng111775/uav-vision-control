#!/usr/bin/env python3
"""Summarize D_TARGET_V2 logs without conflating publishes and frames."""
import argparse,json,statistics
from pathlib import Path
def pct(a,p): return sorted(a)[int((len(a)-1)*p)] if a else None
def main():
 p=argparse.ArgumentParser();p.add_argument('log');p.add_argument('--json',default='performance.json');a=p.parse_args(); rows=[]
 for line in Path(a.log).read_text(errors='ignore').splitlines():
  f=line.split(',')
  if len(f)==12 and f[0]=='D_TARGET_V2': rows.append(f)
 seq=[int(r[1]) for r in rows];proc=[int(r[3])/1000 for r in rows];valid=[r for r in rows if r[5]=='1']
 report={'serial_lines_hz':None,'new_frame_hz':None,'valid_observation_hz':None,'ROS_publish_hz':None,'repeated_publish_hz':None,'processing_ms':{'p50':pct(proc,.5),'p90':pct(proc,.9),'p95':pct(proc,.95),'max':max(proc) if proc else None},'malformed_line_count':0,'dropped_frame_count':sum(max(0,b-a-1) for a,b in zip(seq,seq[1:])),'samples':len(rows),'valid_samples':len(valid)}
 Path(a.json).write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
if __name__=='__main__':main()
