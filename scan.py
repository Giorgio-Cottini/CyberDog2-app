import socket, concurrent.futures, sys
ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.137.115"
lo = int(sys.argv[2]) if len(sys.argv) > 2 else 1
hi = int(sys.argv[3]) if len(sys.argv) > 3 else 10000
def probe(p):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(0.4)
    try:
        if s.connect_ex((ip, p)) == 0:
            return p
    except OSError:
        pass
    finally:
        s.close()
    return None
open_ports = []
with concurrent.futures.ThreadPoolExecutor(max_workers=300) as ex:
    for r in ex.map(probe, range(lo, hi+1)):
        if r: open_ports.append(r)
print("OPEN su %s [%d-%d]: %s" % (ip, lo, hi, sorted(open_ports) if open_ports else "nessuna"))
