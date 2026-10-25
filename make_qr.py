import os, sys, json, base64
from Crypto.Cipher import AES

KEY = "b1950ca92ccc9ffe"  # hard-coded key from cyberdog connector.cpp

def getKey(key):
    n = len(key)
    if n <= 16:   key += '0'*(16-n)
    elif n <= 24: key += '0'*(24-n)
    elif n <= 32: key += '0'*(32-n)
    else:         key = key[:32]
    return key

def encrypt_null(key, text):
    # match dog's decrypt(): AES-ECB, base64; pad plaintext with NUL to 16-byte blocks
    raw = text.encode('utf-8')
    if len(raw) % 16:
        raw += b'\x00' * (16 - len(raw) % 16)
    c = AES.new(getKey(key).encode('utf-8'), AES.MODE_ECB)
    return base64.b64encode(c.encrypt(raw)).decode('ascii')

def dog_decrypt(key, text):
    # exact reproduction of connector/script/aes.py decrypt()
    c = AES.new(getKey(key).encode('utf-8'), AES.MODE_ECB)
    pt = c.decrypt(base64.b64decode(text.encode('utf-8')))
    pt = pt.strip()
    pt = pt.decode('utf-8')
    return pt.strip('\x00')

ssid     = os.environ["DOG_SSID"]
password = os.environ["DOG_PASS"]
provider = os.environ["DOG_PROVIDER"]

plain = '{"n":"%s","p":"%s","i":"%s"}' % (ssid, password, provider)
cipher = encrypt_null(KEY, plain)

# ---- self-test: decrypt exactly like the dog, then validate like connector.cpp ----
back = dog_decrypt(KEY, cipher)
assert ('"n":' in back) and ('"p":' in back) and ('"i":' in back), "key markers missing"
doc = json.loads(back[:back.rindex('}')+1])
assert doc["n"] == ssid and doc["p"] == password and doc["i"] == provider, "round-trip mismatch"
assert len(ssid) <= 32 and len(password) <= 32, "field >32 bytes (dog rejects)"
print("SELF-TEST OK | cipher.len=%d | ssid.len=%d prov=%s" % (len(cipher), len(ssid), provider))

# ---- render QR as scalable SVG inside an HTML page ----
import qrcode
qr = qrcode.QRCode(error_correction=qrcode.constants.ERROR_CORRECT_H, border=4)
qr.add_data(cipher)
qr.make(fit=True)
m = qr.get_matrix()
n = len(m)
rects = []
for y,row in enumerate(m):
    for x,v in enumerate(row):
        if v:
            rects.append('<rect x="%d" y="%d" width="1" height="1"/>' % (x,y))
svg = ('<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 %d %d" '
       'shape-rendering="crispEdges" style="width:min(92vw,92vh);height:min(92vw,92vh);background:#fff">'
       '<rect width="%d" height="%d" fill="#fff"/><g fill="#000">%s</g></svg>') % (n,n,n,n,"".join(rects))

html = ('<!doctype html><meta charset="utf-8"><title>CyberDog QR</title>'
        '<style>html,body{margin:0;height:100%%;display:flex;align-items:center;'
        'justify-content:center;background:#fff}</style>%s' % svg)

out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "qr.html")
with open(out,"w",encoding="utf-8") as f:
    f.write(html)
print("WROTE", out, "| modules=%dx%d" % (n,n))
