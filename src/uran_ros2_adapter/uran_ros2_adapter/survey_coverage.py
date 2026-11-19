"""Observed ray footprints used to bound an optimized SLAM occupancy grid."""
import math
import sqlite3

import numpy as np

from .map_geometry import MAPPING_RADIUS_M


class SurveyCoverage:
    def __init__(self,path,map_id,frame,tile=0.05):
        self.tile = tile
        self.db = sqlite3.connect(path or ":memory:")
        self.db.execute("CREATE TABLE IF NOT EXISTS survey (map_id TEXT, frame TEXT, tile REAL)")
        info = self.db.execute("SELECT * FROM survey").fetchone()
        if info is None:
            self.db.execute("INSERT INTO survey VALUES (?,?,?)",(map_id,frame,tile))
        elif info != (map_id,frame,tile):
            raise ValueError("coverage file belongs to a different map or frame")
        self.db.execute("CREATE TABLE IF NOT EXISTS rays (x INTEGER,y INTEGER,bx REAL,by REAL,PRIMARY KEY(x,y))")
        self.db.commit()
        self.origins = {(x,y):(bx,by) for x,y,bx,by in self.db.execute("SELECT * FROM rays")}
        self.dirty = {}

    def observe(self,origin,ends,body):
        if not len(ends):
            return
        steps = np.ceil(np.linalg.norm(ends-origin,axis=1)/(self.tile*0.5)).astype(int)
        steps = np.maximum(1,steps)
        i = np.arange(steps.max()+1)
        beams,samples = np.nonzero(i[None,:] <= steps[:,None])
        xy = origin+(ends[beams]-origin)*(samples/steps[beams])[:,None]
        tiles = np.unique(np.floor(xy/self.tile).astype(int),axis=0)
        bx,by = float(body[0]),float(body[1])
        for x,y in tiles.tolist():
            key = (x,y)
            old = self.origins.get(key)
            if old:
                cx,cy = (x+.5)*self.tile,(y+.5)*self.tile
                # Interior tiles already have one view that covers every possible grid center.
                previous = math.hypot(cx-old[0],cy-old[1])
                if previous <= MAPPING_RADIUS_M-self.tile/math.sqrt(2) or previous <= math.hypot(cx-bx,cy-by):
                    continue
            self.origins[key] = self.dirty[key] = (bx,by)

    def permits(self,points):
        tiles = np.floor(points/self.tile).astype(int)
        origins = np.array([self.origins.get(tuple(t),(np.inf,np.inf)) for t in tiles.tolist()])
        return np.hypot(points[:,0]-origins[:,0],points[:,1]-origins[:,1]) <= MAPPING_RADIUS_M

    def save(self):
        if self.dirty:
            with self.db:
                self.db.executemany("INSERT OR REPLACE INTO rays VALUES (?,?,?,?)",
                                    ((x,y,bx,by) for (x,y),(bx,by) in self.dirty.items()))
            self.dirty.clear()
