"""Measured ray footprints and ground-relative surfaces in a SLAM map."""
import math
import sqlite3

import numpy as np

from .map_geometry import MAPPING_RADIUS_M, within_survey


def surface_heights(points,ground,band):
    """Estimate a small local ground tilt from nearby metric returns anchored at the foot plane."""
    xy = points[:,:2]-ground[:2]
    heights = points[:,2]-ground[2]
    near = (np.hypot(xy[:,0],xy[:,1])<=1.) & (abs(heights)<=band)
    if near.sum()<12:
        return heights
    # Weight patches equally so dense pixels at one distance do not dominate the fit.
    local,z = xy[near],heights[near]
    _,indices = np.unique(np.floor(local/.10).astype(int),axis=0,return_inverse=True)
    patches = [indices==i for i in range(indices.max()+1)]
    if len(patches)<10:
        return heights
    coords = np.array([np.median(local[m],axis=0) for m in patches])
    z = np.array([np.median(z[m]) for m in patches])
    tilt = np.linalg.lstsq(coords,z,rcond=None)[0]
    inliers = abs(z-coords @ tilt)<=band/3
    if inliers.sum()<max(8,.70*len(coords)):
        return heights
    sample = coords[inliers]
    spread = np.linalg.svd(sample-np.mean(sample,axis=0),compute_uv=False)
    if spread[-1]<.07*math.sqrt(len(sample)):
        return heights
    tilt = np.linalg.lstsq(sample,z[inliers],rcond=None)[0]
    if np.linalg.norm(tilt)>math.tan(math.radians(8)):
        return heights
    return heights-xy @ tilt


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
        self.db.execute("CREATE TABLE IF NOT EXISTS edge_views (x INTEGER,y INTEGER,vx INTEGER,vy INTEGER,bx REAL,by REAL,PRIMARY KEY(x,y,vx,vy))")
        self.db.execute("CREATE TABLE IF NOT EXISTS surfaces (x INTEGER,y INTEGER,score INTEGER,stamp INTEGER,vote INTEGER,bx REAL,by REAL,PRIMARY KEY(x,y))")
        self.surfaces = {(x,y):(score,stamp,vote,bx,by) for x,y,score,stamp,vote,bx,by in self.db.execute("SELECT * FROM surfaces")}
        self.changed_surfaces = set()
        self.db.commit()
        self.origins = {(x,y):(bx,by) for x,y,bx,by in self.db.execute("SELECT * FROM rays")}
        self.dirty = {}
        self.edge_views = {}
        for x,y,vx,vy,bx,by in self.db.execute("SELECT * FROM edge_views"):
            self.edge_views.setdefault((x,y),{})[(vx,vy)] = (bx,by)
        self.new_views = []

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
                full = MAPPING_RADIUS_M-self.tile/math.sqrt(2)
                if previous <= full:
                    continue
                if math.hypot(cx-bx,cy-by) > full:
                    # At the circle edge different viewpoints can cover opposite sides of this tile.
                    view = (math.floor(bx/self.tile),math.floor(by/self.tile))
                    views = self.edge_views.setdefault(key,{})
                    if view not in views:
                        views[view] = (bx,by)
                        self.new_views.append((x,y,view[0],view[1],bx,by))
                    continue
            self.origins[key] = self.dirty[key] = (bx,by)

    def permits(self,points):
        tiles = np.floor(points/self.tile).astype(int)
        origins = np.array([self.origins.get(tuple(t),(np.inf,np.inf)) for t in tiles.tolist()])
        permitted = np.hypot(points[:,0]-origins[:,0],points[:,1]-origins[:,1]) <= MAPPING_RADIUS_M
        for i in np.flatnonzero(~permitted):
            views = self.edge_views.get(tuple(tiles[i]))
            if views:
                x,y = points[i]
                permitted[i] = any(math.hypot(x-bx,y-by) <= MAPPING_RADIUS_M for bx,by in views.values())
        return permitted

    def observe_surfaces(self,beams,ground,ground_band,body_height,stamp):
        # Bin physical endpoints only. A ray towards the floor can pass over an obstacle.
        for group in beams:
            body = group["body_xy"]
            points = np.asarray(group["points"],dtype=float).reshape(-1,3)
            height = surface_heights(points,ground,ground_band)
            inside = within_survey(points,body) & np.isfinite(height)
            floor = inside & (abs(height)<=ground_band)
            raised_or_drop = inside & ((height < -ground_band) | ((height>ground_band) & (height<=body_height)))
            for vote,selected in ((-1,floor),(1,raised_or_drop)):
                if not selected.any():
                    continue
                tiles = np.unique(np.floor(points[selected,:2]/self.tile).astype(int),axis=0)
                for x,y in tiles.tolist():
                    key = (x,y)
                    score,previous,last_vote,_,_ = self.surfaces.get(key,(0,-1,0,0,0))
                    if stamp<previous or (stamp==previous and vote<=last_vote):
                        continue
                    if stamp==previous:
                        score -= last_vote
                    score = max(-3,min(3,score+vote))
                    self.surfaces[key] = (score,stamp,vote,float(body[0]),float(body[1]))
                    self.changed_surfaces.add(key)

    def overlay(self,raster,origin,rotation,resolution):
        """Add measured surfaces, including those outside the planar raster's bounds."""
        stable = [(x,y,score,bx,by) for (x,y),(score,_,_,bx,by) in self.surfaces.items() if abs(score)>=2]
        if not stable:
            return raster,origin
        evidence = np.asarray(stable)
        tiles,occupied,views = evidence[:,:2],evidence[:,2]>0,evidence[:,3:5]
        centers = (tiles+.5)*self.tile
        central = np.floor((centers-origin) @ rotation/resolution).astype(int)
        # A fine or rotated output grid can have several cell centers inside one measured tile.
        reach = int(math.ceil(self.tile/(math.sqrt(2)*resolution)))
        cells,values = [],[]
        for dx in range(-reach,reach+1):
            for dy in range(-reach,reach+1):
                ij = central + (dx,dy)
                xy = ((ij+.5)*resolution) @ rotation.T + origin
                same_tile = np.all(np.floor(xy/self.tile)==tiles,axis=1)
                # Preserve a measured obstacle smaller than an output cell.
                use = (same_tile | (occupied & (dx==0) & (dy==0))) & (np.hypot(xy[:,0]-views[:,0],xy[:,1]-views[:,1])<=MAPPING_RADIUS_M)
                if use.any():
                    cells.append(ij[use])
                    values.append(occupied[use])
        if not cells:
            return raster,origin
        ij = np.concatenate(cells)
        blocked = np.concatenate(values)
        low = np.minimum(ij.min(axis=0),[0,0])
        high = np.maximum(ij.max(axis=0)+1,raster.shape[::-1])
        result = np.full((high-low)[::-1],-1,dtype=raster.dtype)
        h,w = raster.shape
        result[-low[1]:h-low[1],-low[0]:w-low[0]] = raster
        x,y = (ij-low).T
        fill = ~blocked & (result[y,x]<0)
        result[y[fill],x[fill]] = 0
        result[y[blocked],x[blocked]] = 100
        return result, origin + (low*resolution) @ rotation.T

    def save(self):
        if self.dirty or self.new_views or self.changed_surfaces:
            with self.db:
                self.db.executemany("INSERT OR REPLACE INTO rays VALUES (?,?,?,?)",
                                    ((x,y,bx,by) for (x,y),(bx,by) in self.dirty.items()))
                self.db.executemany("INSERT OR IGNORE INTO edge_views VALUES (?,?,?,?,?,?)",self.new_views)
                self.db.executemany("INSERT OR REPLACE INTO surfaces VALUES (?,?,?,?,?,?,?)",
                                    ((x,y)+self.surfaces[(x,y)] for x,y in self.changed_surfaces))
            self.changed_surfaces.clear()
            self.dirty.clear()
            self.new_views.clear()
