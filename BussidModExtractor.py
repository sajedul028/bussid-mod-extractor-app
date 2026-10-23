# -*- coding: utf-8 -*-
"""
BUSSID Mod Extractor
--------------------
Extracts a .bussidmod file (Bus Simulator Indonesia mod) into:
  * textures  -> PNG files (ETC1/KTX decoded, alpha merged)
  * 3D model  -> OBJ + MTL (hierarchy transforms applied)
  * info      -> thumbnail, description, vehicle data (JSON), materials, node tree
and can pack everything into ZIP files.

Run:  python BussidModExtractor.py            (GUI)
      python BussidModExtractor.py file.bussidmod [output_folder]   (command line)
"""
import os, re, sys, json, math, struct, zipfile, threading, queue, traceback
import numpy as np
from PIL import Image

APP_NAME = "BUSSID Mod Extractor"
APP_VERSION = "1.0"

# =====================================================================
#  .NET BinaryFormatter (MS-NRBF) reader
# =====================================================================
PRIM = {1:'bool',2:'byte',3:'char',5:'decimal',6:'double',7:'int16',8:'int32',9:'int64',
        10:'sbyte',11:'single',12:'timespan',13:'datetime',14:'uint16',15:'uint32',
        16:'uint64',18:'string'}
FMT = {'bool':'<?','byte':'<B','sbyte':'<b','int16':'<h','uint16':'<H','int32':'<i',
       'uint32':'<I','int64':'<q','uint64':'<Q','single':'<f','double':'<d'}

class Ref:
    def __init__(self, i): self.id = i

class Obj:
    def __init__(self, cls, members, oid):
        self.cls, self.m, self.id = cls, members, oid

class Reader:
    def __init__(self, data, pos=0):
        self.d, self.p = data, pos
        self.objs, self.classes, self.libs = {}, {}, {}
    def u8(self):
        v = self.d[self.p]; self.p += 1; return v
    def rd(self, n):
        v = self.d[self.p:self.p+n]; self.p += n; return v
    def i32(self):
        v = struct.unpack_from('<i', self.d, self.p)[0]; self.p += 4; return v
    def lps(self):
        n = 0; sh = 0
        while True:
            b = self.u8(); n |= (b & 0x7f) << sh; sh += 7
            if not b & 0x80: break
        return self.rd(n).decode('utf-8', 'replace')
    def prim(self, t):
        if t == 'string': return self.lps()
        if t == 'char':
            b = self.u8(); n = 1 if b < 0x80 else 2 if b < 0xe0 else 3 if b < 0xf0 else 4
            self.p += n - 1; return '?'
        if t in FMT:
            f = FMT[t]; v = struct.unpack_from(f, self.d, self.p)[0]
            self.p += struct.calcsize(f); return v
        if t in ('timespan', 'datetime'):
            v = struct.unpack_from('<q', self.d, self.p)[0]; self.p += 8; return v
        if t == 'decimal': return self.lps()
        raise ValueError('unknown primitive ' + str(t))
    def typeinfo(self, btypes):
        out = []
        for bt in btypes:
            if bt in (0, 7): out.append(PRIM[self.u8()])
            elif bt == 3: out.append(self.lps())
            elif bt == 4: out.append((self.lps(), self.i32()))
            else: out.append(None)
        return out
    def read_class(self, rt):
        if rt == 1:
            oid = self.i32(); mid = self.i32()
            return self.members(self.classes[mid], oid)
        oid = self.i32(); name = self.lps(); n = self.i32()
        names = [self.lps() for _ in range(n)]
        btypes = ti = None
        if rt in (4, 5):
            btypes = [self.u8() for _ in range(n)]; ti = self.typeinfo(btypes)
        if rt in (3, 5): self.i32()
        cl = dict(name=name, names=names, btypes=btypes, ti=ti)
        self.classes[oid] = cl
        return self.members(cl, oid)
    def members(self, cl, oid):
        m = {}
        for i, nm in enumerate(cl['names']):
            if cl['btypes'][i] == 0: m[nm] = self.prim(cl['ti'][i])
            else: m[nm] = self.read_value()
        o = Obj(cl['name'], m, oid); self.objs[oid] = o; return o
    def read_value(self):
        return self.dispatch(self.u8())
    def dispatch(self, rt):
        if rt == 6:
            oid = self.i32(); v = self.lps(); self.objs[oid] = v; return v
        if rt == 9: return Ref(self.i32())
        if rt == 10: return None
        if rt == 13: return ('nulls', self.u8())
        if rt == 14: return ('nulls', self.i32())
        if rt == 8: return self.prim(PRIM[self.u8()])
        if rt in (1, 2, 3, 4, 5): return self.read_class(rt)
        if rt == 12:
            i = self.i32(); self.libs[i] = self.lps(); return self.read_value()
        if rt == 15:
            oid = self.i32(); n = self.i32(); t = PRIM[self.u8()]
            if t == 'byte': v = self.rd(n)
            elif t in FMT:
                f = FMT[t]
                v = list(struct.unpack_from('<%d%s' % (n, f[1]), self.d, self.p))
                self.p += n * struct.calcsize(f)
            else: v = [self.prim(t) for _ in range(n)]
            self.objs[oid] = v; return v
        if rt in (16, 17):
            oid = self.i32(); n = self.i32(); out = []
            while len(out) < n:
                v = self.read_value()
                if isinstance(v, tuple) and v and v[0] == 'nulls': out.extend([None] * v[1])
                else: out.append(v)
            self.objs[oid] = out; return out
        if rt == 7:
            oid = self.i32(); at = self.u8(); rank = self.i32()
            lens = [self.i32() for _ in range(rank)]
            if at in (3, 4, 5):
                for _ in range(rank): self.i32()
            bt = self.u8(); ti = self.typeinfo([bt])[0]
            total = 1
            for l in lens: total *= l
            out = []
            while len(out) < total:
                if bt == 0: out.append(self.prim(ti))
                else:
                    v = self.read_value()
                    if isinstance(v, tuple) and v and v[0] == 'nulls': out.extend([None] * v[1])
                    else: out.append(v)
            self.objs[oid] = out; return out
        if rt == 11: return 'END'
        raise ValueError('unknown record type %d at offset %d' % (rt, self.p - 1))
    def parse(self):
        if self.u8() != 0: raise ValueError('not a BinaryFormatter stream')
        for _ in range(4): self.i32()
        root = None
        while True:
            v = self.read_value()
            if v == 'END': break
            if root is None: root = v
        return root
    def R(self, v):
        while isinstance(v, Ref): v = self.objs[v.id]
        return v

STREAM_SIG = b'\x00\x01\x00\x00\x00\xff\xff\xff\xff\x01\x00\x00\x00'

def parse_streams(data, log):
    """Parse every serialized stream stored back-to-back in the file."""
    streams = []; pos = 0
    while pos < len(data):
        if data[pos:pos+len(STREAM_SIG)] != STREAM_SIG:
            nxt = data.find(STREAM_SIG, pos, pos + 256)
            if nxt < 0: break
            pos = nxt
        r = Reader(data, pos)
        root = r.parse()
        streams.append((pos, r.p, r, root))
        log('  stream @%d..%d : %s' % (pos, r.p, root.cls if isinstance(root, Obj) else type(root).__name__))
        pos = r.p
    return streams

def plain(r, v, depth=0):
    v = r.R(v)
    if depth > 25: return '...'
    if isinstance(v, Obj): return {k: plain(r, x, depth + 1) for k, x in v.m.items() if k != 'Comparer'}
    if isinstance(v, (bytes, bytearray)): return '<bytes %d>' % len(v)
    if isinstance(v, list): return [plain(r, x, depth + 1) for x in v]
    if isinstance(v, tuple): return None
    return v

# =====================================================================
#  ETC1 decoder (vectorised)
# =====================================================================
_MOD = np.array([[2,8],[5,17],[9,29],[13,42],[18,60],[24,80],[33,106],[47,183]], dtype=np.int64)

def decode_etc1(data, w, h):
    bw, bh = w // 4, h // 4; n = bw * bh
    raw = np.frombuffer(data, dtype=np.uint8, count=n * 8).reshape(n, 8).astype(np.int64)
    hi = ((raw[:,0] << 24) | (raw[:,1] << 16) | (raw[:,2] << 8) | raw[:,3]) & 0xffffffff
    lo = ((raw[:,4] << 24) | (raw[:,5] << 16) | (raw[:,6] << 8) | raw[:,7]) & 0xffffffff
    d = ((hi >> 1) & 1) == 1
    flip = hi & 1
    t1 = (hi >> 5) & 7; t2 = (hi >> 2) & 7
    r1 = (hi >> 28) & 15; r2 = (hi >> 24) & 15
    g1 = (hi >> 20) & 15; g2 = (hi >> 16) & 15
    b1 = (hi >> 12) & 15; b2 = (hi >> 8) & 15
    sext = lambda x: np.where(x >= 4, x - 8, x)
    e5 = lambda x: (x << 3) | (x >> 2)
    R1 = np.where(d, e5((hi >> 27) & 31), (r1 << 4) | r1)
    G1 = np.where(d, e5((hi >> 19) & 31), (g1 << 4) | g1)
    B1 = np.where(d, e5((hi >> 11) & 31), (b1 << 4) | b1)
    R2 = np.where(d, e5(np.clip(((hi >> 27) & 31) + sext((hi >> 24) & 7), 0, 31)), (r2 << 4) | r2)
    G2 = np.where(d, e5(np.clip(((hi >> 19) & 31) + sext((hi >> 16) & 7), 0, 31)), (g2 << 4) | g2)
    B2 = np.where(d, e5(np.clip(((hi >> 11) & 31) + sext((hi >> 8) & 7), 0, 31)), (b2 << 4) | b2)
    base = np.stack([np.stack([R1, G1, B1], 1), np.stack([R2, G2, B2], 1)], 1)
    tcw = np.stack([t1, t2], 1)
    px = np.arange(16)
    idx = (((lo[:, None] >> (px[None, :] + 16)) & 1) << 1) | ((lo[:, None] >> px[None, :]) & 1)
    xs = px // 4; ys = px % 4
    sub = np.where((flip[:, None] == 1), (ys >= 2)[None, :], (xs >= 2)[None, :]).astype(np.int64)
    mods = _MOD[np.take_along_axis(tcw, sub, 1)]
    small = mods[..., 0]; big = mods[..., 1]
    mval = np.where(idx == 0, small, np.where(idx == 1, big, np.where(idx == 2, -small, -big)))
    basep = np.take_along_axis(base, sub[:, :, None].repeat(3, 2), 1)
    col = np.clip(basep + mval[:, :, None], 0, 255).astype(np.uint8)
    col = col.reshape(bh, bw, 4, 4, 3).transpose(0, 3, 1, 2, 4)
    return col.reshape(h, w, 3)

def decode_texture(b):
    """Return ('img', ndarray) | ('raw', bytes, ext) | ('ktx', bytes, '')."""
    b = bytes(b)
    if b[:8] == b'\x89PNG\r\n\x1a\n': return ('raw', b, '.png')
    if b[:3] == b'\xff\xd8\xff': return ('raw', b, '.jpg')
    if b[:12] == b'\xabKTX 11\xbb\r\n\x1a\n':
        endian = struct.unpack_from('<I', b, 12)[0]
        if endian == 0x04030201:
            (gltype, _gts, glfmt, glint, _glbase, w, h, _d, _a, _f, _m, kv) = struct.unpack_from('<12I', b, 16)
            off = 64 + kv
            isz = struct.unpack_from('<I', b, off)[0]; data = b[off + 4: off + 4 + isz]
            if glint == 0x8D64:                      # GL_ETC1_RGB8_OES
                return ('img', decode_etc1(data, w, h))
            if gltype == 0x1401 and glfmt in (0x1907, 0x1908):
                ch = 3 if glfmt == 0x1907 else 4
                return ('img', np.frombuffer(data, np.uint8, w * h * ch).reshape(h, w, ch))
    return ('ktx', b, '.ktx')

# =====================================================================
#  Helpers
# =====================================================================
def sanitize(n):
    n = re.sub(r'[^\w\-.]+', '_', (n or '').strip())
    return n or 'item'

def dictitems(r, dobj):
    dobj = r.R(dobj)
    if dobj is None: return []
    kv = r.R(dobj.m['KeyValuePairs']); out = []
    for e in (kv or []):
        e = r.R(e)
        if e is None: continue
        out.append((r.R(e.m['key']), r.R(e.m['value'])))
    return out

def qmat(q):
    x, y, z, w = q; n = math.sqrt(x*x + y*y + z*z + w*w) or 1
    x /= n; y /= n; z /= n; w /= n
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def write_json(path, obj):
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(obj, f, indent=1, ensure_ascii=False)

# =====================================================================
#  Main extraction
# =====================================================================
def extract(src, out_root, opts, log=print, progress=lambda p, t='': None):
    do_tex = opts.get('textures', True); do_model = opts.get('model', True)
    do_info = opts.get('info', True); do_zip = opts.get('zip', True)
    base = sanitize(os.path.splitext(os.path.basename(src))[0])
    out = os.path.join(out_root, base + '_extracted')
    os.makedirs(out, exist_ok=True)
    for s in ('info', 'textures', 'models'):
        os.makedirs(os.path.join(out, s), exist_ok=True)

    progress(2, 'ফাইল পড়া হচ্ছে...')
    log('Reading: ' + src)
    data = open(src, 'rb').read()
    log('File size: %.2f MB' % (len(data) / 1048576))

    progress(5, 'ডেটা পার্স করা হচ্ছে...')
    log('Parsing data blocks...')
    streams = parse_streams(data, log)
    if not streams: raise ValueError('এটা বৈধ .bussidmod ফাইল বলে মনে হচ্ছে না।')

    model_root = model_r = None
    summary = {'textures': 0, 'meshes': 0, 'objects': 0}
    for pos, end, r, root in streams:
        cname = root.cls if isinstance(root, Obj) else ''
        if cname == 'ModelFile.MFModel':
            model_r, model_root = r, root; continue
        if not do_info: continue
        if cname == 'ModFile+Header':
            hm = {k: r.R(v) for k, v in root.m.items()}
            thumb = hm.get('thumbnail')
            if thumb:
                open(os.path.join(out, 'info', 'thumbnail.jpg'), 'wb').write(bytes(thumb))
            with open(os.path.join(out, 'info', 'description.txt'), 'w', encoding='utf-8') as f:
                f.write('Title: %s\nAuthor: %s\nVersion: %s\nType: %s\n\n%s\n' % (
                    hm.get('title'), hm.get('author'), hm.get('version'), hm.get('type'), hm.get('description')))
            log('Saved info: thumbnail + description')
        else:
            fname = sanitize(cname.split('+')[-1].split('.')[-1]).lower() or 'data'
            fname = {'vehicleheader': 'vehicle_header', 'vehicledata': 'vehicle_data'}.get(fname, fname)
            write_json(os.path.join(out, 'info', fname + '.json'), plain(r, root))
            log('Saved info: %s.json' % fname)

    if model_root is None:
        log('⚠ এই ফাইলে 3D মডেল/টেক্সচার ব্লক পাওয়া যায়নি।')
    else:
        r = model_r; R = r.R
        # ---------------- textures ----------------
        progress(15, 'টেক্সচার ডিকোড হচ্ছে...')
        alpha = {k: v for k, v in dictitems(r, model_root.m['alphaTextures'])}
        texfile = {}; used = set(); tlog = []
        tex_items = dictitems(r, model_root.m['textures'])
        for i, (k, t) in enumerate(tex_items):
            name = R(t.m['nameLCase']) or ('tex_%s' % k)
            stem = os.path.splitext(sanitize(name))[0]
            res = decode_texture(R(t.m['bytes']))
            if res[0] == 'img':
                fn = stem + '.png'
            else:
                fn = stem + res[2]
            if fn in used: fn = '%s_%d%s' % (os.path.splitext(fn)[0], abs(k), os.path.splitext(fn)[1])
            used.add(fn)
            if do_tex or do_model:
                path = os.path.join(out, 'textures', fn)
                if res[0] == 'img':
                    arr = res[1]; mode = 'RGB' if arr.shape[2] == 3 else 'RGBA'
                    if k in alpha and arr.shape[2] == 3:
                        ares = decode_texture(R(alpha[k].m['bytes']))
                        if ares[0] == 'img':
                            a = ares[1][:, :, 0]
                            if a.shape[:2] != arr.shape[:2]:
                                a = np.array(Image.fromarray(a).resize((arr.shape[1], arr.shape[0])))
                            arr = np.dstack([arr, a]); mode = 'RGBA'
                    arr = np.ascontiguousarray(arr[::-1])     # GL bottom-left -> normal top-left
                    Image.fromarray(arr, mode).save(path)
                    tlog.append('%s (%dx%d, %s)  original: %s' % (fn, arr.shape[1], arr.shape[0], mode, name))
                else:
                    open(path, 'wb').write(res[1]); tlog.append('%s (raw)  original: %s' % (fn, name))
            texfile[k] = fn; summary['textures'] += 1
            progress(15 + 40 * (i + 1) / max(1, len(tex_items)), 'টেক্সচার %d/%d' % (i + 1, len(tex_items)))
        if do_info:
            open(os.path.join(out, 'info', 'texture_list.txt'), 'w', encoding='utf-8').write('\n'.join(tlog) + '\n')
        log('Textures written: %d' % summary['textures'])

        # ---------------- materials ----------------
        mats = {}
        for k, m in dictitems(r, model_root.m['materials']):
            cols = {ck: [R(cv).m[a] for a in 'xyzw'] for ck, cv in dictitems(r, m.m['colors'])}
            flo = {fk: fv for fk, fv in dictitems(r, m.m['floats'])}
            tx = {tk: tv for tk, tv in dictitems(r, m.m['textures'])}
            vec = {vk: [R(vv).m[a] for a in 'xyzw'] for vk, vv in dictitems(r, m.m['vectors'])}
            mats[k] = dict(name=R(m.m['name']), shader=R(m.m['shaderName']), colors=cols, floats=flo,
                           vectors=vec, textures={s: texfile.get(i, i) for s, i in tx.items()})
        if do_info:
            write_json(os.path.join(out, 'info', 'materials.json'), {str(k): v for k, v in mats.items()})

        # ---------------- model ----------------
        if do_model:
            progress(58, 'মেশ লোড হচ্ছে...')
            meshes = {}
            for k, mm in dictitems(r, model_root.m['meshes']):
                v = np.array(R(mm.m['vertices']), dtype=np.float64).reshape(-1, 3)
                nr = R(mm.m['normals']); n = np.array(nr, dtype=np.float64).reshape(-1, 3) if nr else None
                uv = R(mm.m['uv1']); u = np.array(uv, dtype=np.float64).reshape(-1, 2) if uv else None
                subs = [np.array(R(R(s).m['triangles']), dtype=np.int64).reshape(-1, 3) for s in R(mm.m['submeshes'])]
                meshes[k] = (v, n, u, subs)
            summary['meshes'] = len(meshes)
            log('Meshes: %d' % len(meshes))

            # MTL
            used_m = set(); mtlname = {}; lines = []
            for k, m in mats.items():
                nm = sanitize(m['name'] or 'mat_%s' % k)
                if nm in used_m: nm = '%s_%d' % (nm, abs(k))
                used_m.add(nm); mtlname[k] = nm
                c = m['colors'].get('_Color', [1, 1, 1, 1])
                lines += ['newmtl ' + nm, 'Ka 0 0 0', 'Kd %.4f %.4f %.4f' % (c[0], c[1], c[2]), 'Ks 0 0 0']
                if c[3] < 1: lines.append('d %.4f' % c[3])
                tex = m['textures']; main = tex.get('_MainTex') or next(iter(tex.values()), None)
                if isinstance(main, str): lines.append('map_Kd ../textures/' + main)
                lines += ['# shader: %s' % m['shader'], '']
            open(os.path.join(out, 'models', 'model.mtl'), 'w').write('\n'.join(lines))

            FIX = np.eye(4); FIX[:3, :3] = [[1, 0, 0], [0, 0, 1], [0, -1, 0]]   # Z-up -> Y-up
            tree = []; counter = {'v': 1, 'vt': 1, 'vn': 1, 'objs': 0}
            progress(65, 'OBJ লেখা হচ্ছে...')
            fobj = open(os.path.join(out, 'models', 'model.obj'), 'w', buffering=1 << 20)
            fobj.write('# Extracted by %s\nmtllib model.mtl\n' % APP_NAME)

            def walk(g, parentM, depth):
                g = R(g); name = R(g.m['name']) or 'node'
                p = R(g.m['position']); q = R(g.m['rotation']); s = R(g.m['localScale'])
                M = np.eye(4)
                M[:3, :3] = qmat([q.m[a] for a in 'xyzw']) @ np.diag([s.m[a] for a in 'xyz'])
                M[:3, 3] = [p.m[a] for a in 'xyz']
                W = parentM @ M
                mid = g.m['meshId']; mids = R(g.m['materialsIds']); matlist = []
                if isinstance(mids, Obj):
                    matlist = [R(x) for x in R(mids.m['_items'])[:R(mids.m['_size'])]]
                elif isinstance(mids, list): matlist = [R(x) for x in mids]
                tree.append('  ' * depth + name + ('  [mesh %s]' % mid if mid in meshes else ''))
                if mid in meshes:
                    v, n, u, subs = meshes[mid]
                    vw = (W[:3, :3] @ v.T).T + W[:3, 3]; vw[:, 0] *= -1
                    fobj.write('o %s\n' % sanitize(name))
                    fobj.write(''.join('v %.6f %.6f %.6f\n' % tuple(a) for a in vw))
                    if u is not None: fobj.write(''.join('vt %.6f %.6f\n' % tuple(a) for a in u))
                    if n is not None:
                        nw = (np.linalg.inv(W[:3, :3]).T @ n.T).T; nw[:, 0] *= -1
                        l = np.linalg.norm(nw, axis=1, keepdims=True); l[l == 0] = 1; nw = nw / l
                        fobj.write(''.join('vn %.5f %.5f %.5f\n' % tuple(a) for a in nw))
                    rev = np.linalg.det(W[:3, :3]) > 0
                    vo, vto, vno = counter['v'], counter['vt'], counter['vn']
                    for si, tri in enumerate(subs):
                        mk = matlist[si] if si < len(matlist) else None
                        if mk in mtlname: fobj.write('usemtl %s\n' % mtlname[mk])
                        if rev: tri = tri[:, [0, 2, 1]]
                        out_l = []
                        for a, b_, c_ in tri:
                            def f(i):
                                if u is not None and n is not None: return '%d/%d/%d' % (vo + i, vto + i, vno + i)
                                if u is not None: return '%d/%d' % (vo + i, vto + i)
                                if n is not None: return '%d//%d' % (vo + i, vno + i)
                                return str(vo + i)
                            out_l.append('f %s %s %s\n' % (f(a), f(b_), f(c_)))
                        fobj.write(''.join(out_l))
                    counter['v'] += len(v)
                    if u is not None: counter['vt'] += len(u)
                    if n is not None: counter['vn'] += len(n)
                    counter['objs'] += 1
                    progress(65 + 22 * min(1, counter['objs'] / max(1, len(meshes))), 'মডেল %d/%d' % (counter['objs'], len(meshes)))
                ch = R(g.m['children'])
                if isinstance(ch, Obj):
                    for c in R(ch.m['_items'])[:R(ch.m['_size'])]:
                        if c is not None: walk(c, W, depth + 1)
            for k, g in dictitems(r, model_root.m['gameObjects']):
                walk(g, FIX, 0)
            fobj.close()
            summary['objects'] = counter['objs']
            if do_info:
                open(os.path.join(out, 'info', 'hierarchy.txt'), 'w', encoding='utf-8').write('\n'.join(tree) + '\n')
            log('OBJ written: %d objects, %d vertices' % (counter['objs'], counter['v'] - 1))

    # remove empty dirs
    for s in ('info', 'textures', 'models'):
        p = os.path.join(out, s)
        if os.path.isdir(p) and not os.listdir(p): os.rmdir(p)

    zips = []
    if do_zip:
        progress(90, 'ZIP বানানো হচ্ছে...')
        def make_zip(zpath, subdirs):
            with zipfile.ZipFile(zpath, 'w', zipfile.ZIP_DEFLATED, compresslevel=6) as z:
                for sd in subdirs:
                    root_dir = os.path.join(out, sd)
                    if not os.path.isdir(root_dir): continue
                    for dp, _, fns in os.walk(root_dir):
                        for fn in fns:
                            full = os.path.join(dp, fn)
                            z.write(full, os.path.join(os.path.basename(out), os.path.relpath(full, out)))
            zips.append(zpath); log('ZIP created: ' + os.path.basename(zpath))
        make_zip(os.path.join(out_root, base + '_ALL_files.zip'), ['info', 'textures', 'models'])
        make_zip(os.path.join(out_root, base + '_Textures_and_3D_Model.zip'), ['textures', 'models'])

    progress(100, 'সম্পন্ন!')
    log('\nDONE ✔  Output folder: ' + out)
    return out, summary, zips

# =====================================================================
#  GUI
# =====================================================================
def run_gui():
    import tkinter as tk
    from tkinter import ttk, filedialog, messagebox

    BG = '#f3f5f9'; ACCENT = '#2563eb'; DARK = '#111827'
    root = tk.Tk(); root.title('%s v%s' % (APP_NAME, APP_VERSION))
    root.geometry('800x680'); root.minsize(720, 600); root.configure(bg=BG)
    style = ttk.Style(root)
    try: style.theme_use('clam')
    except Exception: pass
    style.configure('TFrame', background=BG)
    style.configure('Card.TLabelframe', background=BG, padding=10)
    style.configure('Card.TLabelframe.Label', background=BG, foreground=DARK, font=('Segoe UI', 10, 'bold'))
    style.configure('TLabel', background=BG, foreground=DARK, font=('Segoe UI', 10))
    style.configure('TCheckbutton', background=BG, font=('Segoe UI', 10))
    style.configure('Start.TButton', font=('Segoe UI', 11, 'bold'), foreground='white', background=ACCENT, padding=8)
    style.map('Start.TButton', background=[('active', '#1d4ed8'), ('disabled', '#9ca3af')])
    style.configure('TButton', font=('Segoe UI', 10), padding=5)
    style.configure('Blue.Horizontal.TProgressbar', troughcolor='#e5e7eb', background=ACCENT, thickness=18)

    header = tk.Frame(root, bg=DARK); header.pack(fill='x')
    tk.Label(header, text='🚌  BUSSID Mod Extractor', bg=DARK, fg='white', font=('Segoe UI', 18, 'bold')).pack(anchor='w', padx=20, pady=(14, 0))
    tk.Label(header, text='.bussidmod ফাইল থেকে টেক্সচার (PNG), 3D মডেল (OBJ) ও অন্যান্য ফাইল বের করুন',
             bg=DARK, fg='#cbd5e1', font=('Segoe UI', 10)).pack(anchor='w', padx=22, pady=(0, 14))

    body = ttk.Frame(root); body.pack(fill='both', expand=True, padx=18, pady=12)

    src_var = tk.StringVar(); out_var = tk.StringVar()
    o_tex = tk.BooleanVar(value=True); o_model = tk.BooleanVar(value=True)
    o_info = tk.BooleanVar(value=True); o_zip = tk.BooleanVar(value=True)

    f1 = ttk.LabelFrame(body, text=' 1. মড ফাইল (.bussidmod) ', style='Card.TLabelframe'); f1.pack(fill='x')
    e1 = ttk.Entry(f1, textvariable=src_var, font=('Segoe UI', 10)); e1.pack(side='left', fill='x', expand=True, padx=(0, 8))
    def pick_src():
        p = filedialog.askopenfilename(title='.bussidmod ফাইল বাছাই করুন',
                                       filetypes=[('BUSSID Mod', '*.bussidmod'), ('All files', '*.*')])
        if p:
            src_var.set(p)
            if not out_var.get(): out_var.set(os.path.join(os.path.dirname(p), 'Extracted'))
    ttk.Button(f1, text='📂 Browse', command=pick_src).pack(side='left')

    f2 = ttk.LabelFrame(body, text=' 2. আউটপুট ফোল্ডার ', style='Card.TLabelframe'); f2.pack(fill='x', pady=(10, 0))
    ttk.Entry(f2, textvariable=out_var, font=('Segoe UI', 10)).pack(side='left', fill='x', expand=True, padx=(0, 8))
    def pick_out():
        p = filedialog.askdirectory(title='আউটপুট ফোল্ডার বাছাই করুন')
        if p: out_var.set(p)
    ttk.Button(f2, text='📁 Browse', command=pick_out).pack(side='left')

    f3 = ttk.LabelFrame(body, text=' 3. কী কী বের করবেন ', style='Card.TLabelframe'); f3.pack(fill='x', pady=(10, 0))
    ttk.Checkbutton(f3, text='টেক্সচার (PNG)', variable=o_tex).grid(row=0, column=0, sticky='w', padx=6, pady=2)
    ttk.Checkbutton(f3, text='3D মডেল (OBJ + MTL)', variable=o_model).grid(row=0, column=1, sticky='w', padx=24, pady=2)
    ttk.Checkbutton(f3, text='ইনফো ফাইল (থাম্বনেইল, JSON, বর্ণনা)', variable=o_info).grid(row=1, column=0, sticky='w', padx=6, pady=2)
    ttk.Checkbutton(f3, text='ZIP ফাইল বানান (সব + টেক্সচার/মডেল আলাদা)', variable=o_zip).grid(row=1, column=1, sticky='w', padx=24, pady=2)

    btns = ttk.Frame(body); btns.pack(fill='x', pady=12)
    start_btn = ttk.Button(btns, text='▶  Extract শুরু করুন', style='Start.TButton'); start_btn.pack(side='left')
    open_btn = ttk.Button(btns, text='📂 আউটপুট ফোল্ডার খুলুন', state='disabled'); open_btn.pack(side='left', padx=10)

    pb = ttk.Progressbar(body, style='Blue.Horizontal.TProgressbar', maximum=100); pb.pack(fill='x')
    status = tk.StringVar(value='প্রস্তুত'); ttk.Label(body, textvariable=status).pack(anchor='w', pady=(4, 6))

    logf = ttk.LabelFrame(body, text=' লগ ', style='Card.TLabelframe'); logf.pack(fill='both', expand=True)
    txt = tk.Text(logf, height=10, wrap='word', bg='#0b1220', fg='#d1fae5', insertbackground='white',
                  font=('Consolas', 9), relief='flat'); sb = ttk.Scrollbar(logf, command=txt.yview)
    txt.configure(yscrollcommand=sb.set, state='disabled')
    txt.pack(side='left', fill='both', expand=True); sb.pack(side='right', fill='y')

    q = queue.Queue(); state = {'out': None}

    def append(s):
        txt.configure(state='normal'); txt.insert('end', s + '\n'); txt.see('end'); txt.configure(state='disabled')

    def poll():
        try:
            while True:
                kind, a, b = q.get_nowait()
                if kind == 'log': append(a)
                elif kind == 'prog': pb['value'] = a; status.set(b or status.get())
                elif kind == 'done':
                    start_btn.configure(state='normal')
                    state['out'] = a['out']; open_btn.configure(state='normal')
                    s = a['summary']
                    messagebox.showinfo('সম্পন্ন', 'এক্সট্রাক্ট সফল হয়েছে!\n\nটেক্সচার: %d\nমেশ/অবজেক্ট: %d / %d\n\nফোল্ডার:\n%s' %
                                        (s['textures'], s['meshes'], s['objects'], a['out']))
                elif kind == 'err':
                    start_btn.configure(state='normal'); status.set('ত্রুটি হয়েছে')
                    append(a); messagebox.showerror('ত্রুটি', b)
        except queue.Empty:
            pass
        root.after(100, poll)

    def worker(src, outdir, opts):
        try:
            out, summary, _ = extract(src, outdir, opts,
                                      log=lambda s: q.put(('log', s, None)),
                                      progress=lambda p, t='': q.put(('prog', p, t)))
            q.put(('done', {'out': out, 'summary': summary}, None))
        except Exception as e:
            q.put(('err', traceback.format_exc(), 'সমস্যা: %s' % e))

    def start():
        src = src_var.get().strip(); outdir = out_var.get().strip()
        if not src or not os.path.isfile(src):
            messagebox.showwarning('ফাইল নেই', 'আগে একটি .bussidmod ফাইল বাছাই করুন।'); return
        if not outdir:
            outdir = os.path.join(os.path.dirname(src), 'Extracted'); out_var.set(outdir)
        if not (o_tex.get() or o_model.get() or o_info.get()):
            messagebox.showwarning('কিছু বাছাই করুন', 'অন্তত একটি অপশন সিলেক্ট করুন।'); return
        os.makedirs(outdir, exist_ok=True)
        txt.configure(state='normal'); txt.delete('1.0', 'end'); txt.configure(state='disabled')
        pb['value'] = 0; start_btn.configure(state='disabled'); open_btn.configure(state='disabled')
        opts = dict(textures=o_tex.get(), model=o_model.get(), info=o_info.get(), zip=o_zip.get())
        threading.Thread(target=worker, args=(src, outdir, opts), daemon=True).start()

    def open_folder():
        p = state['out']
        if not p: return
        try:
            if sys.platform.startswith('win'): os.startfile(os.path.dirname(p))
            elif sys.platform == 'darwin': os.system('open "%s"' % os.path.dirname(p))
            else: os.system('xdg-open "%s"' % os.path.dirname(p))
        except Exception: pass

    start_btn.configure(command=start); open_btn.configure(command=open_folder)
    tk.Label(root, text='Tip: বড় মডে কয়েক মিনিট লাগতে পারে — উইন্ডো বন্ধ করবেন না।', bg=BG, fg='#6b7280',
             font=('Segoe UI', 9)).pack(pady=(0, 8))
    poll(); root.mainloop()

def main():
    if len(sys.argv) >= 2:
        src = sys.argv[1]
        outdir = sys.argv[2] if len(sys.argv) > 2 else os.path.join(os.path.dirname(os.path.abspath(src)), 'Extracted')
        os.makedirs(outdir, exist_ok=True)
        extract(src, outdir, dict(textures=True, model=True, info=True, zip=True),
                log=print, progress=lambda p, t='': print('[%3d%%] %s' % (p, t)) if int(p) % 10 == 0 else None)
    else:
        run_gui()

if __name__ == '__main__':
    main()
