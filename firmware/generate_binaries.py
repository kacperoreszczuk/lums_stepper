from http.server import HTTPServer, SimpleHTTPRequestHandler
import socket
from socketserver import ThreadingMixIn
import threading
import shutil
import os
import traceback
import time

import ifcfg
import psutil
import subprocess
import threading


token = b"uniquetoken45AD43383042B227439E97405EF5A"

def createBinary(node, source, output):
    binary = bytearray(open(source, 'rb').read())
    position = binary.find(token)
    if position == -1:
        raise Exception("Token not found in .bin!")

    position -= 2

    oldXor = binary[position] ^ binary[position + 1]
    binary[position] = node - 100
    
    # update checksum byte to keep .bin file checksum correct
    binary[position + 1] = oldXor ^ binary[position]

    open(output, "bw").write(binary)

def prepareAllImages():
    build = False
    try:
        os.mkdir("binaries_all")
    except FileExistsError:
        pass
    for i in range(101, 200):
        createBinary(i, source="build/servos.bin", output=f"binaries_all/stepper-{i}.bin")


def main():
    prepareAllImages()

if __name__ == '__main__':
    main()