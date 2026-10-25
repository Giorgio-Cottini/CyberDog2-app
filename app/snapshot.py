#!/usr/bin/env python3
# Pull a single still photo from the dog over the FORWARD channel only.
# Uses GrpcApp.getFile(SendRequest{4002, {"command":1}}) -> stream FileChunk.
# No reverse channel, no cyberdog2.client cert needed.
import json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
CDOG = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(CDOG, "pylibs"))
sys.path.insert(0, os.path.join(CDOG, "proto"))
import grpc
import cyberdog_app_pb2 as pb
import cyberdog_app_pb2_grpc as rpc

IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.137.115"
PORT = sys.argv[2] if len(sys.argv) > 2 else "50052"
CMD = int(sys.argv[3]) if len(sys.argv) > 3 else 1   # 1 = photo
OUT = sys.argv[4] if len(sys.argv) > 4 else os.path.join(HERE, "snapshot.jpg")
CERTS = os.path.join(CDOG, "certs")


def rd(p):
    with open(p, "rb") as f:
        return f.read()


creds = grpc.ssl_channel_credentials(
    root_certificates=rd(os.path.join(CERTS, "ca-cert.pem")),
    private_key=rd(os.path.join(CERTS, "client-key.pem")),
    certificate_chain=rd(os.path.join(CERTS, "client-cert.pem")))
opts = (("grpc.ssl_target_name_override", "cyberdog2.server"),
        ("grpc.default_authority", "cyberdog2.server"),
        ("grpc.max_receive_message_length", 32 * 1024 * 1024))
ch = grpc.secure_channel("%s:%s" % (IP, PORT), creds, options=opts)
stub = rpc.GrpcAppStub(ch)
req = pb.SendRequest(nameCode=4002, params=json.dumps({"command": CMD}))

buf = bytearray()
name = None
size = None
err = None
try:
    for chunk in stub.getFile(req, timeout=30):
        err = chunk.error_code
        name = chunk.file_name
        size = chunk.file_size
        if chunk.buffer:
            buf += chunk.buffer
except grpc.RpcError as e:
    print("RPC error:", e.code(), e.details())
    sys.exit(2)

print("error_code=%s file_name=%s file_size=%s received=%d" % (err, name, size, len(buf)))
if buf:
    with open(OUT, "wb") as f:
        f.write(buf)
    print("wrote", OUT, len(buf), "bytes")
else:
    print("no bytes received")
