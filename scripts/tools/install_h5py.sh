#!/bin/bash
# ~/.ssh/config 别名（LAN 不可达时自动回退 Tailscale）；未配置时可改回 rm@192.168.0.115
ssh rm "sudo docker exec xrobo-vr-teleop bash -c 'pip install h5py pytest 2>&1'"
