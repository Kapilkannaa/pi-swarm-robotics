"""Query new firmware; discovery never sends motor commands or feeds watchdogs."""
import argparse
import json
import ipaddress
import socket
import time


def discover(host="255.255.255.255", seconds=2.0, subnet=None):
    found = {}
    hosts = [host]
    if subnet:
        network = ipaddress.ip_network(subnet, strict=False)
        if network.version != 4 or network.num_addresses > 256 or not network.is_private:
            raise ValueError("use a private IPv4 subnet of /24 or smaller")
        hosts = [str(ip) for ip in network.hosts()]
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        s.settimeout(0.2)
        for _ in range(3):
            for address in hosts:
                s.sendto(b"SWARM_DISCOVER", (address, 4210))
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            try:
                raw, addr = s.recvfrom(1024)
                d = json.loads(raw)
                if d.get("project") == "pi-swarm-v1":
                    d["source_ip"] = addr[0]
                    found[(d["id"], addr[0])] = d
            except (socket.timeout, ConnectionResetError, ValueError, KeyError, AttributeError):
                continue
    return list(found.values())


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="255.255.255.255", help="use hotspot broadcast or known bot IP if needed")
    ap.add_argument("--subnet", help="private /24 or smaller; unicast fallback when hotspot blocks broadcast")
    a = ap.parse_args()
    print(json.dumps(discover(a.host, subnet=a.subnet), indent=2))
