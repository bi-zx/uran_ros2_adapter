"""Measured ray footprints and ground-relative surfaces in a SLAM map."""
import math
import sqlite3

import numpy as np
from scipy.ndimage import label

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
    def __init__(self,path,map_id,frame,tile=0.05,surface_confirmation_s=1.0):
        self.tile = tile
        self.surface_confirmation_ns = int(surface_confirmation_s*1e9)
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
        if "candidate_height" not in {row[1] for row in self.db.execute("PRAGMA table_info(surfaces)")}:
            self.db.execute("ALTER TABLE surfaces ADD COLUMN candidate_height REAL")
        if "corroborated" not in {row[1] for row in self.db.execute("PRAGMA table_info(surfaces)")}:
            self.db.execute("ALTER TABLE surfaces ADD COLUMN corroborated INTEGER DEFAULT 1")
        self.surfaces,self.candidate_heights,self.corroborated = {},{},set()
        for x,y,score,stamp,vote,bx,by,z,confirmed in self.db.execute("SELECT x,y,score,stamp,vote,bx,by,candidate_height,corroborated FROM surfaces"):
            if confirmed and score>=2:
                self.corroborated.add((x,y))
            self.surfaces[(x,y)] = (score,stamp,vote,bx,by)
            if z is not None:
                self.candidate_heights[(x,y)] = z
        self.db.execute("CREATE TABLE IF NOT EXISTS scan_evidence (x INTEGER,y INTEGER,hits INTEGER,crossings INTEGER,stamp INTEGER,PRIMARY KEY(x,y))")
        if "occupancy" not in {row[1] for row in self.db.execute("PRAGMA table_info(scan_evidence)")}:
            self.db.execute("ALTER TABLE scan_evidence ADD COLUMN occupancy INTEGER")
        self.scan_evidence = {(x,y):(hits,crossings,stamp,state) for x,y,hits,crossings,stamp,state in
                              self.db.execute("SELECT x,y,hits,crossings,stamp,occupancy FROM scan_evidence")}
        # The latest scan is enough to continue a two-frame confirmation after a restart.
        self.last_scan_stamp = max((v[2] for v in self.scan_evidence.values()),default=-1)
        self.changed_scans = {}
        self.changed_surfaces = set()
        self.db.commit()
        self.origins = {(x,y):(bx,by) for x,y,bx,by in self.db.execute("SELECT * FROM rays")}
        self.dirty = {}
        self.edge_views = {}
        for x,y,vx,vy,bx,by in self.db.execute("SELECT * FROM edge_views"):
            self.edge_views.setdefault((x,y),{})[(vx,vy)] = (bx,by)
        self.new_views = []

    def observe(self,origin,ends,body,stamp=None):
        ends = np.asarray(ends,dtype=float).reshape(-1,2)
        ends = ends[np.isfinite(ends).all(axis=1) & (np.hypot(ends[:,0]-body[0],ends[:,1]-body[1]) <= MAPPING_RADIUS_M)]
        if not len(ends):
            return
        steps = np.ceil(np.linalg.norm(ends-origin,axis=1)/(self.tile*0.5)).astype(int)
        steps = np.maximum(1,steps)
        i = np.arange(steps.max()+1)
        beams,samples = np.nonzero(i[None,:] <= steps[:,None])
        xy = origin+(ends[beams]-origin)*(samples/steps[beams])[:,None]
        indices = np.floor(xy/self.tile).astype(np.int64)
        first_x,first_y = map(int,indices.min(axis=0))
        width = int(indices[:,0].max())-first_x+1
        ids = (indices[:,1]-first_y)*width + indices[:,0]-first_x
        tiles = np.unique(ids)
        bx,by = float(body[0]),float(body[1])
        hits = set()
        if stamp is not None:
            end_tiles = np.floor(ends/self.tile).astype(np.int64)
            hits = set(((end_tiles[:,1]-first_y)*width+end_tiles[:,0]-first_x).tolist())
        crossings = set()
        if stamp is not None:
            # Use interior crossings with a return beyond the cell. Grazing a corner,
            # terminating in the cell, or a missing return does not measure it empty.
            centers = (indices+.5)*self.tile
            delta = ends[beams]-origin
            length2 = np.einsum("ij,ij->i",delta,delta)
            t = np.einsum("ij,ij->i",centers-origin,delta)/np.maximum(length2,1e-20)
            offset = centers-(origin+t[:,None]*delta)
            interior = ((t>=0) & ((1-t)*np.sqrt(length2)>self.tile/math.sqrt(2)) &
                        (np.max(np.abs(offset),axis=1)<self.tile/4) &
                        (np.hypot(centers[:,0]-bx,centers[:,1]-by)<=MAPPING_RADIUS_M))
            crossings = set(np.unique(ids[interior]).tolist())
        for index in tiles.tolist():
            row,col = divmod(index,width)
            x,y = first_x+col,first_y+row
            key = (x,y)
            if stamp is not None and (index in hits or index in crossings):
                count,passed,previous,state = self.scan_evidence.get(key,(0,0,-1,None))
                if stamp>previous and stamp>=self.last_scan_stamp:
                    if index in hits:
                        count = 2 if count and state is not None and previous==self.last_scan_stamp else 1
                        state = 100 if count==2 else (-1 if state is None else state)
                        passed = 0
                    else:
                        count,passed,state = 0,min(3,passed+1),0
                    record = (count,passed,stamp,state)
                    self.changed_scans[key] = self.scan_evidence[key] = record
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
        if stamp is not None:
            self.last_scan_stamp = max(self.last_scan_stamp,stamp)

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
            acquired = group.get("timestamp_ns",stamp)
            height = surface_heights(points,np.asarray(group.get("ground_xyz",ground)),ground_band)
            inside = within_survey(points,body) & np.isfinite(height)
            floor = inside & (abs(height)<=ground_band)
            raised_or_drop = inside & ((height < -ground_band) | ((height>ground_band) & (height<=body_height)))
            for vote,selected in ((-1,floor),(1,raised_or_drop)):
                if not selected.any():
                    continue
                xy = np.floor(points[selected,:2]/self.tile).astype(int)
                if vote>0:
                    heights = height[selected]
                    order = np.lexsort((heights,xy[:,1],xy[:,0]))
                    xy,heights = xy[order],heights[order]
                tiles,starts,counts = np.unique(xy,axis=0,return_index=True,return_counts=True)
                for (x,y),first,count in zip(tiles.tolist(),starts,counts):
                    key = (x,y)
                    score,previous,last_vote,_,_ = self.surfaces.get(key,(0,-1,0,0,0))
                    if acquired<previous or (acquired==previous and vote<=last_vote):
                        continue
                    if acquired==previous:
                        score -= last_vote
                    if vote>0:
                        z = heights[first+count//2]
                        candidate = self.candidate_heights.get(key)
                        if score==1 and (candidate is None or np.min(np.abs(heights[first:first+count]-candidate))>ground_band):
                            score = 0
                        if score>=2 or (score==1 and last_vote==1 and 0<acquired-previous<=self.surface_confirmation_ns):
                            self.corroborated.add(key)
                        self.candidate_heights[key] = z
                    score = max(-3,min(3,score+vote))
                    if score<2:
                        self.corroborated.discard(key)
                    self.surfaces[key] = (score,acquired,vote,float(body[0]),float(body[1]))
                    self.changed_surfaces.add(key)

    def _filter_scan_speckles(self,raster,origin,rotation,resolution):
        """Show isolated single-scan returns as unknown until corroborated."""
        if not self.scan_evidence:
            return raster
        components,n = label(raster>=65,structure=np.ones((3,3),np.uint8))
        if not n:
            return raster
        sizes = np.bincount(components.ravel(),minlength=n+1)
        sizes[0] = 4
        rows,cols = np.where(sizes[components]<=3)
        if not len(rows):
            return raster
        # Protect a small component as a whole, including returns on cell boundaries.
        local_xy = (np.column_stack((cols,rows))+.5)*resolution
        centers = local_xy @ rotation.T + origin
        tiles = np.floor(centers/self.tile).astype(int)
        votes = np.asarray([self.scan_evidence.get(tuple(t),(0,0,0))[:2] for t in tiles.tolist()])
        uncertain = (votes[:,0]==1) | (votes[:,1]>=3)
        protected = set(components[rows[~uncertain],cols[~uncertain]].tolist())
        reach = int(math.ceil(resolution/(math.sqrt(2)*self.tile)))
        for dx in range(-reach,reach+1):
            for dy in range(-reach,reach+1):
                nearby = tiles+(dx,dy)
                hit_centers = (nearby+.5)*self.tile
                delta = (hit_centers-centers) @ rotation
                # Tile squares may rotate relative to the occupancy grid.
                overlap = np.all(abs(delta) < resolution/2+self.tile*.5*np.sum(abs(rotation),axis=0)-1e-10,axis=1)
                for index in np.flatnonzero(overlap):
                    if self.scan_evidence.get(tuple(nearby[index]),(0,0,0))[0]>=2:
                        protected.add(components[rows[index],cols[index]])
        remove = ~np.isin(components[rows,cols],list(protected))
        if remove.any():
            raster = raster.copy()
            raster[rows[remove],cols[remove]] = -1
        return raster

    def overlay(self,raster,origin,rotation,resolution):
        """Add measured surfaces, including those outside the planar raster's bounds."""
        # Ground can fill an unknown cell. A measured planar crossing can clear an
        # old wall. A confirmed raised surface or drop takes precedence at any height.
        stable = []
        for (x,y),(score,_,_,bx,by) in self.surfaces.items():
            if score>=2:
                # A lone depth return needs a consistent pair close in time, a third
                # observation, or an adjacent independently measured raised/drop tile.
                if (x,y) not in self.corroborated and not any(self.surfaces.get((x+dx,y+dy),(0,))[0]>=2
                        for dx,dy in ((-1,0),(1,0),(0,-1),(0,1))):
                    continue
                stable.append((x,y,3,bx,by))
            elif score<=-2:
                stable.append((x,y,0,bx,by))
        for cell,(_,_,_,state) in self.scan_evidence.items():
            if state in (0,100) and cell in self.origins:
                stable.append((*cell,2 if state==100 else 1,*self.origins[cell]))
        if not stable:
            return self._filter_scan_speckles(raster,origin,rotation,resolution),origin
        evidence = np.asarray(stable)
        tiles,kinds,views = evidence[:,:2],evidence[:,2],evidence[:,3:5]
        occupied = kinds>=2
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
                in_range = np.hypot(xy[:,0]-views[:,0],xy[:,1]-views[:,1])<=MAPPING_RADIUS_M
                edge = (~in_range) & ((kinds==1) | (kinds==2))
                if edge.any():
                    in_range[edge] = self.permits(xy[edge])
                use = (same_tile | (occupied & (dx==0) & (dy==0))) & in_range
                if use.any():
                    cells.append(ij[use])
                    values.append(kinds[use])
        if not cells:
            return self._filter_scan_speckles(raster,origin,rotation,resolution),origin
        ij = np.concatenate(cells)
        kinds = np.concatenate(values)
        low = np.minimum(ij.min(axis=0),[0,0])
        high = np.maximum(ij.max(axis=0)+1,raster.shape[::-1])
        result = np.full((high-low)[::-1],-1,dtype=raster.dtype)
        h,w = raster.shape
        result[-low[1]:h-low[1],-low[0]:w-low[0]] = raster
        x,y = (ij-low).T
        result[y[kinds==1],x[kinds==1]] = 0
        result[y[kinds==2],x[kinds==2]] = 100
        origin = origin + (low*resolution) @ rotation.T
        result = self._filter_scan_speckles(result,origin,rotation,resolution)
        floor = (kinds==0) & (result[y,x]<0)
        result[y[floor],x[floor]] = 0
        result[y[kinds==3],x[kinds==3]] = 100
        return result,origin

    def save(self):
        if self.dirty or self.new_views or self.changed_surfaces or self.changed_scans:
            with self.db:
                self.db.executemany("INSERT OR REPLACE INTO scan_evidence(x,y,hits,crossings,stamp,occupancy) VALUES (?,?,?,?,?,?)",
                                    (cell+value for cell,value in self.changed_scans.items()))
                self.db.executemany("INSERT OR REPLACE INTO rays VALUES (?,?,?,?)",
                                    ((x,y,bx,by) for (x,y),(bx,by) in self.dirty.items()))
                self.db.executemany("INSERT OR IGNORE INTO edge_views VALUES (?,?,?,?,?,?)",self.new_views)
                self.db.executemany("INSERT OR REPLACE INTO surfaces(x,y,score,stamp,vote,bx,by,candidate_height,corroborated) VALUES (?,?,?,?,?,?,?,?,?)",
                                    (key+self.surfaces[key]+(self.candidate_heights.get(key),key in self.corroborated)
                                     for key in self.changed_surfaces))
            self.changed_scans.clear()
            self.changed_surfaces.clear()
            self.dirty.clear()
            self.new_views.clear()
