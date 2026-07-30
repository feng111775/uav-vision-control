"""Pure geometry state for fast detector; no sensor/image dependency."""
def bounded_roi(cx, cy, diameter, width, height, scale, lost=0):
    half=max(16, int(diameter * scale * (1.35 ** lost) / 2))
    x=max(0, int(cx-half)); y=max(0, int(cy-half)); x2=min(width,int(cx+half)); y2=min(height,int(cy+half))
    return (x,y,max(0,x2-x),max(0,y2-y)) if x2-x>=24 and y2-y>=24 else None

class TrackState:
    def __init__(self): self.last=None;self.lost=0;self.frame=0
    def roi(self,image,scale):
        return None if self.last is None else bounded_roi(self.last['cx'],self.last['cy'],self.last['outer_diameter_px'],image.width(),image.height(),scale,self.lost)
    def update(self,result):
        self.frame+=1
        if result and result.get('valid'): self.last=result;self.lost=0
        else:self.lost+=1
