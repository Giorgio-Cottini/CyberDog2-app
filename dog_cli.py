import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "proto"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import grpc
import cyberdog_app_pb2 as pb
import cyberdog_app_pb2_grpc as rpc

ip   = sys.argv[1] if len(sys.argv) > 1 else "192.168.137.1"
port = sys.argv[2] if len(sys.argv) > 2 else "8981"
code = int(sys.argv[3]) if len(sys.argv) > 3 else 1001   # 1001 GET_DEVICE_INFO (read-only)
params = sys.argv[4] if len(sys.argv) > 4 else "{}"

target = "%s:%s" % (ip, port)
authority = os.environ.get("DOG_AUTHORITY", "cyberdog2.server")
ca_path    = os.environ.get("DOG_CA")          # trust root (ca-cert.pem)
cert_path  = os.environ.get("DOG_CLIENTCERT")  # our client chain (server-cert.pem)
key_path   = os.environ.get("DOG_CLIENTKEY")   # our client key   (server-key.pem)

def _read(p):
    with open(p, "rb") as f:
        return f.read()

if ca_path and cert_path and key_path:
    mode = "mtls"
    creds = grpc.ssl_channel_credentials(
        root_certificates=_read(ca_path),
        private_key=_read(key_path),
        certificate_chain=_read(cert_path))
    opts = (("grpc.ssl_target_name_override", authority),
            ("grpc.default_authority", authority))
    ch = grpc.secure_channel(target, creds, options=opts)
elif ca_path:
    mode = "tls"
    creds = grpc.ssl_channel_credentials(root_certificates=_read(ca_path))
    opts = (("grpc.ssl_target_name_override", authority),
            ("grpc.default_authority", authority))
    ch = grpc.secure_channel(target, creds, options=opts)
else:
    mode = "plain"
    ch = grpc.insecure_channel(target)
print("connecting to", target, "nameCode", code, "params", params, "mode", mode)
try:
    grpc.channel_ready_future(ch).result(timeout=5)
    print("channel READY")
except grpc.FutureTimeoutError:
    print("channel NOT ready (no server on %s)" % target); sys.exit(2)

stub = rpc.GrpcAppStub(ch)
req = pb.SendRequest(nameCode=code, params=params)
try:
    got = False
    for rsp in stub.sendMsg(req, timeout=8):
        got = True
        print("RESP nameCode=%s data=%s" % (rsp.nameCode, rsp.data))
    if not got:
        print("stream empty (server accepted but returned nothing)")
except grpc.RpcError as e:
    print("RPC error:", e.code(), e.details())
