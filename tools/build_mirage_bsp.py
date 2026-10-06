#!/usr/bin/env python3
"""Convert a GoldSrc BSP into textured GLB geometry plus spawn/floor/collision metadata.
Usage: python3 tools/build_mirage_bsp.py /path/to/de_mirage_cs2.bsp
No third-party Python packages are required.
"""
from __future__ import annotations
import json, math, pathlib, struct, zlib, sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
BSP = pathlib.Path(sys.argv[1]) if len(sys.argv)>1 else pathlib.Path('assets/de_mirage_cs2.bsp')
OUT = ROOT / 'assets/mirage_bsp.glb'
SCALE = 118.0 / 6080.0
CENTER_X, CENTER_Y = -128.0, 144.0
FLOOR_Z = 88.0

def png_rgba(w, h, rgba):
    raw = b''.join(b'\0' + rgba[y*w*4:(y+1)*w*4] for y in range(h))
    def chunk(tag, data):
        return struct.pack('>I', len(data)) + tag + data + struct.pack('>I', zlib.crc32(tag+data)&0xffffffff)
    return b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB',w,h,8,6,0,0,0)) + chunk(b'IDAT',zlib.compress(raw,6)) + chunk(b'IEND',b'')

def main():
    if not BSP.is_file(): raise SystemExit(f'BSP not found: {BSP}\nPass the input path as the first argument.')
    data=BSP.read_bytes()
    version=struct.unpack_from('<i',data)[0]
    if version != 30: raise SystemExit(f'Expected GoldSrc BSP v30, got {version}')
    lumps=[struct.unpack_from('<ii',data,4+i*8) for i in range(15)]
    def lump(i):
        o,n=lumps[i]; return data[o:o+n]
    verts_raw=lump(3); vertices=[struct.unpack_from('<3f',verts_raw,i) for i in range(0,len(verts_raw),12)]
    edges_raw=lump(12); edges=[struct.unpack_from('<2H',edges_raw,i) for i in range(0,len(edges_raw),4)]
    se_raw=lump(13); surfedges=[struct.unpack_from('<i',se_raw,i)[0] for i in range(0,len(se_raw),4)]
    ti_raw=lump(6); texinfo=[struct.unpack_from('<8fii',ti_raw,i) for i in range(0,len(ti_raw),40)]
    faces_raw=lump(7); faces=[struct.unpack_from('<Hhihh4si',faces_raw,i) for i in range(0,len(faces_raw),20)]
    planes_raw=lump(1); planes=[struct.unpack_from('<3ffI',planes_raw,i) for i in range(0,len(planes_raw),20)]
    nodes_raw=lump(5); nodes=[struct.unpack_from('<i2h3h3h2H',nodes_raw,i) for i in range(0,(len(nodes_raw)//24)*24,24)]
    leaves_raw=lump(10); leaves=[struct.unpack_from('<ii3h3h2H4B',leaves_raw,i) for i in range(0,(len(leaves_raw)//28)*28,28)]
    def point_contents(x,y,z):
        node=0
        for _ in range(512):
            if node<0: return leaves[-node-1][0] if -node-1<len(leaves) else -1
            if node>=len(nodes): return -1
            q=nodes[node]; pl=planes[q[0]]; side=0 if x*pl[0]+y*pl[1]+z*pl[2]>=pl[3] else 1; node=q[1+side]
        return -1
    mt=lump(2); count=struct.unpack_from('<i',mt)[0]; miptex=[]
    for i in range(count):
        off=struct.unpack_from('<i',mt,4+i*4)[0]
        if off<0: miptex.append(None); continue
        name=mt[off:off+16].split(b'\0')[0].decode('latin1',errors='replace')
        w,h=struct.unpack_from('<II',mt,off+16); offs=struct.unpack_from('<4I',mt,off+24)
        pixels=mt[off+offs[0]:off+offs[0]+w*h]
        paloff=off+offs[3]+max(1,w//8)*max(1,h//8)
        rgba=None
        if len(pixels)==w*h and paloff+2<=len(mt):
            colors=struct.unpack_from('<H',mt,paloff)[0]; p=paloff+2
            if colors>=256 and p+768<=len(mt):
                pal=mt[p:p+768]; out=bytearray(w*h*4)
                for j,k in enumerate(pixels): out[j*4:j*4+4]=bytes((pal[k*3],pal[k*3+1],pal[k*3+2],255))
                rgba=bytes(out)
        if not rgba:
            # External/missing WAD texture: restrained sandstone fallback, never magenta.
            hsh=sum(name.encode())
            base=(157+(hsh%26),132+(hsh%22),96+(hsh%18))
            rgba=bytes((*base,255))*max(1,w*h)
        miptex.append((name,w,h,png_rgba(w,h,rgba)))
    # World model only. Coordinates are converted from GoldSrc Z-up to Three.js Y-up.
    model=struct.unpack_from('<9f7i',lump(14),0); firstface,numfaces=model[-2:]
    groups={}; skipped=0; walls=[]; floor_cells={}; cell=.5; grid_min=-59.0
    for f in faces:
        _,_,firstedge,nedges,ti,_,_=f
        if ti<0 or ti>=len(texinfo) or nedges<3: skipped+=1; continue
        tv=texinfo[ti]; texid=tv[8]
        if texid<0 or texid>=len(miptex): skipped+=1; continue
        tex=miptex[texid]
        if not tex: skipped+=1; continue
        name,w,h,_=tex
        if name.lower()=='sky': continue
        ids=[]
        for se in surfedges[firstedge:firstedge+nedges]:
            edge=edges[abs(se)]; ids.append(edge[0] if se>=0 else edge[1])
        if any(i>=len(vertices) for i in ids): skipped+=1; continue
        arr=groups.setdefault(texid,{'p':[],'uv':[],'n':[],'idx':[]})
        base=len(arr['p'])
        poly=[vertices[i] for i in ids]
        if len(poly)>=3:
            aa,bb,cc=poly[0],poly[1],poly[2]; u=(bb[0]-aa[0],bb[1]-aa[1],bb[2]-aa[2]); v=(cc[0]-aa[0],cc[1]-aa[1],cc[2]-aa[2])
            nn=(u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]); ll=math.sqrt(sum(q*q for q in nn)) or 1; nn=tuple(q/ll for q in nn); worldn=(nn[0],nn[2],-nn[1])
        else: worldn=(0,1,0)
        # Bake walkable top faces into a half-metre floor lookup and vertical faces into collision AABBs.
        pn,pside= f[0],f[1]; normal=planes[pn][:3] if pn<len(planes) else (0,0,0); face_nz=normal[2]*(-1 if pside else 1)
        if face_nz > .72:
            minx=min(v[0] for v in poly); maxx=max(v[0] for v in poly); miny=min(v[1] for v in poly); maxy=max(v[1] for v in poly); zz=sum(v[2] for v in poly)/len(poly)
            ix0=max(0,int(math.floor(((minx-CENTER_X)*SCALE-grid_min)/cell))); ix1=min(235,int(math.floor(((maxx-CENTER_X)*SCALE-grid_min)/cell)))
            iz0=max(0,int(math.floor((-(maxy-CENTER_Y)*SCALE-grid_min)/cell))); iz1=min(235,int(math.floor((-(miny-CENTER_Y)*SCALE-grid_min)/cell)))
            for iz in range(iz0,iz1+1):
                wz=grid_min+(iz+.5)*cell; sy=CENTER_Y-wz/SCALE
                for ix in range(ix0,ix1+1):
                    wx=grid_min+(ix+.5)*cell; sx=CENTER_X+wx/SCALE; inside=False; j=len(poly)-1
                    for k in range(len(poly)):
                        xi,yi=poly[k][0],poly[k][1]; xj,yj=poly[j][0],poly[j][1]
                        if ((yi>sy)!=(yj>sy)) and sx<(xj-xi)*(sy-yi)/(yj-yi+1e-20)+xi: inside=not inside
                        j=k
                    if inside:
                        key=str(iz*236+ix); floor_cells.setdefault(key,set()).add(round(max(0,(zz-FLOOR_Z)*SCALE),3))
        elif abs(normal[2]) < .28:
            xs=[(v[0]-CENTER_X)*SCALE for v in poly]; zs=[-(v[1]-CENTER_Y)*SCALE for v in poly]; ys=[(v[2]-FLOOR_Z)*SCALE for v in poly]
            x0,x1=min(xs),max(xs); z0,z1=min(zs),max(zs); y0,y1=min(ys),max(ys)
            if y1-y0>1.35 and min(x1-x0,z1-z0)<1.25 and max(x1-x0,z1-z0)>.25:
                walls.append([round(x0-.18,3),round(z0-.18,3),round(x1+.18,3),round(z1+.18,3),round(y0,3),round(y1,3)])
        for x,y,z in poly:
            arr['p'].append(((x-CENTER_X)*SCALE,(z-FLOOR_Z)*SCALE,-(y-CENTER_Y)*SCALE))
            s=tv[0]*x+tv[1]*y+tv[2]*z+tv[3]; t=tv[4]*x+tv[5]*y+tv[6]*z+tv[7]
            arr['uv'].append((s/w,t/h)); arr['n'].append(worldn)
        for k in range(1,len(poly)-1): arr['idx'].extend((base,base+k,base+k+1))
    # Build embedded images and a standards-compliant glTF 2.0 binary container.
    binbuf=bytearray(); views=[]; accessors=[]
    def add_blob(blob,target=None):
        while len(binbuf)%4: binbuf.append(0)
        off=len(binbuf); binbuf.extend(blob); v={'buffer':0,'byteOffset':off,'byteLength':len(blob)}
        if target: v['target']=target
        views.append(v); return len(views)-1
    def add_accessor(blob,component,count,typ,target=None,mins=None,maxs=None):
        view=add_blob(blob,target); a={'bufferView':view,'componentType':component,'count':count,'type':typ}
        if mins is not None: a['min']=mins; a['max']=maxs
        accessors.append(a); return len(accessors)-1
    images=[]; textures=[]; materials=[]; mesh_prims=[]
    for texid,g in groups.items():
        tex=miptex[texid]; name,w,h,png=tex
        imgview=add_blob(png)
        images.append({'name':name,'bufferView':imgview,'mimeType':'image/png'})
        textures.append({'sampler':0,'source':len(images)-1})
        materials.append({'name':name,'pbrMetallicRoughness':{'baseColorTexture':{'index':len(textures)-1},'metallicFactor':0.0,'roughnessFactor':0.94},'doubleSided':True})
        p=g['p']; uv=g['uv']; idx=g['idx']
        pblob=b''.join(struct.pack('<3f',*v) for v in p)
        uvblob=b''.join(struct.pack('<2f',*v) for v in uv)
        nblob=b''.join(struct.pack('<3f',*v) for v in g['n'])
        iblob=b''.join(struct.pack('<I',i) for i in idx)
        pa=add_accessor(pblob,5126,len(p),'VEC3',34962,[min(v[k] for v in p) for k in range(3)],[max(v[k] for v in p) for k in range(3)])
        ua=add_accessor(uvblob,5126,len(uv),'VEC2',34962)
        na=add_accessor(nblob,5126,len(g['n']),'VEC3',34962)
        ia=add_accessor(iblob,5125,len(idx),'SCALAR',34963)
        mesh_prims.append({'attributes':{'POSITION':pa,'TEXCOORD_0':ua,'NORMAL':na},'indices':ia,'material':len(materials)-1,'mode':4})
    # Sample BSP solid/empty leaves above every walkable cell for accurate player and projectile collision.
    # Bake the BSP collision tree into an absolute 3D half-metre voxel grid.
    # Unlike face AABBs, this preserves real openings and blocks jumps through solid walls.
    height_min=-4.0; height_step=.5; height_count=48; occupancy={}
    for iz in range(236):
        sy=CENTER_Y-(grid_min+(iz+.5)*cell)/SCALE
        for ix in range(236):
            sx=CENTER_X+(grid_min+(ix+.5)*cell)/SCALE; mask=0
            for k in range(height_count):
                wz=FLOOR_Z+(height_min+.25+k*height_step)/SCALE
                if point_contents(sx,sy,wz)==-2: mask |= (1<<k)
            if mask: occupancy[str(iz*236+ix)]=mask
    # Store exact team spawn points, snapped to the highest nearby walkable BSP face.
    ent=lump(0).decode('latin1',errors='replace')
    spawn={'CT':[],'T':[]}
    for raw in ent.split('}'):
        if 'info_player_start' not in raw and 'info_player_deathmatch' not in raw: continue
        def field(key):
            import re
            m=re.search(r'"'+key+r'"\s+"([^"]*)"',raw); return m.group(1) if m else ''
        cls=field('classname'); org=field('origin')
        try: x,y,z=map(float,org.split())
        except Exception: continue
        key='CT' if cls=='info_player_start' else 'T'
        gx=(x-CENTER_X)*SCALE; gz=-(y-CENTER_Y)*SCALE; gy=max(0,(z-36.0-FLOOR_Z)*SCALE)
        ix=max(0,min(235,int((gx-grid_min)/cell))); iz=max(0,min(235,int((gz-grid_min)/cell)))
        vals=sorted(floor_cells.get(str(iz*236+ix),()))
        floor=min((v for v in vals if v<=gy+.8),key=lambda v:abs(v-gy),default=(max(vals) if vals and min(vals)>gy+.8 else gy))
        spawn[key].append({'x':round(gx,4),'y':round(floor,4),'z':round(gz,4)})
    models=struct.unpack_from('<9f7i',lump(14),0)
    sites={}
    for label,mi in [('A',3),('B',4)]:
        mm=struct.unpack_from('<9f7i',lump(14),mi*64); xx=(mm[0]+mm[3])*.5; yy=(mm[1]+mm[4])*.5
        sites[label]={'x':round((xx-CENTER_X)*SCALE,4),'z':round(-(yy-CENTER_Y)*SCALE,4)}
    extras={'source':'User-provided de_mirage_cs2.bsp (GoldSrc v30)','coordinateTransform':{'scale':SCALE,'centerX':CENTER_X,'centerY':CENTER_Y,'floorZ':FLOOR_Z},'spawns':spawn,'sites':sites,'collisionGrid':{'cell':cell,'min':grid_min,'size':236,'floors':{k:sorted(v) for k,v in floor_cells.items()},'occupancy':occupancy,'heightMin':height_min,'heightStep':height_step,'heightCount':height_count}}
    gltf={'asset':{'version':'2.0','generator':'Melord Mirage BSP converter'},'scene':0,'scenes':[{'nodes':[0]}],
          'nodes':[{'name':'de_mirage_cs2_world','mesh':0,'extras':extras}], 'meshes':[{'name':'Mirage BSP world','primitives':mesh_prims}],
          'materials':materials,'textures':textures,'images':images,'samplers':[{'magFilter':9729,'minFilter':9987,'wrapS':10497,'wrapT':10497}],
          'buffers':[{'byteLength':0}],'bufferViews':views,'accessors':accessors,
          }
    js=json.dumps(gltf,separators=(',',':')).encode(); js+=b' '*((-len(js))%4)
    while len(binbuf)%4: binbuf.append(0)
    gltf['buffers'][0]['byteLength']=len(binbuf)
    js=json.dumps(gltf,separators=(',',':')).encode(); js+=b' '*((-len(js))%4)
    total=12+8+len(js)+8+len(binbuf)
    OUT.write_bytes(struct.pack('<III',0x46546C67,2,total)+struct.pack('<I4s',len(js),b'JSON')+js+struct.pack('<I4s',len(binbuf),b'BIN\0')+binbuf)
    print(f'Output: {OUT} ({OUT.stat().st_size:,} bytes)')
    print(f'World faces: {len(faces):,} (BSP faces total); textured groups: {len(groups)}; skipped: {skipped}')
    print(f'Spawns: CT={len(spawn["CT"])}, T={len(spawn["T"])}; floor cells={len(floor_cells)}; solid voxel columns={len(occupancy)}; sites={sites}')
    print('CT:',spawn['CT']); print('T:',spawn['T'])
if __name__=='__main__': main()
