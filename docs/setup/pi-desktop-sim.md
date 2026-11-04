# Desktop Gazebo SITL with ROS 2 on Raspberry Pi

This setup runs PX4 SITL and Gazebo Harmonic on the Ubuntu 24.04 desktop. The Raspberry Pi runs ROS 2 Jazzy application nodes and the Micro XRCE-DDS Agent. PX4 SITL connects to the Agent over UDP port `8888`. Use either a direct Ethernet link or the existing private router/access point LAN.

The project pins PX4 and `px4_msgs` to v1.17.0 and the Micro XRCE-DDS Agent to v2.4.3. Use matching versions on both machines. See [HITL Pi Setup](HITL-PI-SETUP.md) for the Raspberry Pi build environment.

| Machine | Direct-link address | Processes |
| --- | --- | --- |
| Desktop | `192.168.50.1/24` | PX4 SITL and Gazebo Harmonic |
| Raspberry Pi | `192.168.50.2/24` | Micro XRCE-DDS Agent and ROS 2 nodes |

The addresses in the table apply to the direct Ethernet option. The `/24` prefix is the subnet mask `255.255.255.0`. Choose a different unused private subnet if either machine already uses `192.168.50.0/24` on another interface. Do not configure a gateway or DNS server for the direct Ethernet connection.

## Option A: Direct Ethernet link

### Configure Ethernet addresses

Connect the Ethernet cable. Find each Ethernet interface name with:

```bash
nmcli device status
ip -br link
```

#### NetworkManager

Use NetworkManager on each machine when `nmcli` is installed. Replace `enp3s0` with the Ethernet interface name shown on that machine.

On the desktop:

```bash
IFACE=enp3s0
sudo nmcli connection add type ethernet ifname "$IFACE" \
  con-name pi-desktop-sim ipv4.method manual \
  ipv4.addresses 192.168.50.1/24 ipv4.never-default yes
sudo nmcli connection up pi-desktop-sim
```

On the Raspberry Pi:

```bash
IFACE=enp3s0
sudo nmcli connection add type ethernet ifname "$IFACE" \
  con-name pi-desktop-sim ipv4.method manual \
  ipv4.addresses 192.168.50.2/24 ipv4.never-default yes
sudo nmcli connection up pi-desktop-sim
```

`ipv4.never-default yes` prevents this link from becoming the default route. The connection profile persists after reboot.

#### Netplan on Ubuntu Server

Use Netplan on the Raspberry Pi when NetworkManager and `nmcli` are unavailable. Edit the existing Netplan file for the Ethernet interface. Replace `eth0` with the interface name shown by `ip -br link`.

```yaml
network:
  version: 2
  ethernets:
    eth0:
      dhcp4: false
      addresses:
        - 192.168.50.2/24
      optional: true
```

Apply and verify the configuration:

```bash
sudo netplan try
sudo netplan apply
```

Use `192.168.50.1/24` in the desktop Ethernet interface configuration.

#### Verify the direct link

Run these commands on the desktop:

```bash
ip -4 addr
ping -c 3 192.168.50.2
```

Run these commands on the Raspberry Pi:

```bash
ip -4 addr
ping -c 3 192.168.50.1
```

If UFW is active on the Raspberry Pi, allow incoming UDP port `8888` from the desktop on the Ethernet interface. Replace `eth0` with the Raspberry Pi interface name.

```bash
sudo ufw allow in on eth0 from 192.168.50.1 to any port 8888 proto udp
```

## Option B: Existing router or access point

Connect the desktop and Raspberry Pi to the same private LAN. The desktop can use Wi-Fi or Ethernet. The Raspberry Pi can use Wi-Fi or Ethernet. Use the same non-guest LAN on both devices. The router must allow devices on that LAN to communicate with each other.

Create DHCP reservations for the desktop and Raspberry Pi in the router's DHCP settings. The router associates each reservation with that interface's MAC address. Use `ip link show` or the router's connected-device list to identify the MAC addresses. Keep both network interfaces configured for automatic DHCP. Reserve the Pi address used by PX4. Reserve the desktop address if the Pi firewall rule is limited to a source address.

Find each assigned address with the router's connected-device list or these commands:

```bash
ip -4 -br addr
ip link show
ip route
```

Use the Pi address assigned to the interface connected to the shared LAN. Replace the example addresses below with the router-assigned addresses.

On the desktop:

```bash
ping -c 3 192.168.1.25
ip route get 192.168.1.25
```

On the Raspberry Pi:

```bash
ping -c 3 192.168.1.20
ip route get 192.168.1.20
```

In these examples, `192.168.1.25` is the Pi address and `192.168.1.20` is the desktop address. The route output identifies the interface used to reach the other machine.

If UFW is active on the Raspberry Pi, allow incoming UDP port `8888` from the desktop LAN address. Replace `wlan0` with the Pi's active Wi-Fi or Ethernet interface and replace `192.168.1.20` with the desktop address.

```bash
sudo ufw allow in on wlan0 from 192.168.1.20 to any port 8888 proto udp
```

Start the Raspberry Pi Agent and ROS 2 nodes using the commands in the next section. On the desktop, start PX4 SITL with `./scripts/px4-sitl`. Use the Pi's reserved LAN address in the PX4 client command in the desktop section below.

The simulation container uses host networking and can route to the Pi over the desktop's active LAN interface. PX4 uXRCE-DDS topics have no DDS authentication. Keep this setup on a private LAN and use the local-only DDS settings in the security section below. Guest Wi-Fi and access-point client isolation can block communication between the desktop and Raspberry Pi.

## Start the Raspberry Pi processes

Open a terminal on the Raspberry Pi and start the Agent:

```bash
source /opt/ros/jazzy/setup.bash
MicroXRCEAgent udp4 -p 8888
```

Keep the Agent running. Open a second terminal and source the ROS and workspace overlays. Replace `~/DroneSwarming` with the checkout path on the Pi.

```bash
cd ~/DroneSwarming
source /opt/ros/jazzy/setup.bash
source "$HOME/px4_ros2/install/setup.bash"
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch drone_bringup single_vehicle.launch.py enable_flight_commands:=false
```

`single_vehicle.launch.py` starts the `vehicle_control` and `mission_execution` nodes. Flight commands remain disabled with `enable_flight_commands:=false`.

## Start desktop PX4 SITL and Gazebo

Run the PX4 SITL wrapper from the DroneSwarming checkout on the desktop:

```bash
./scripts/px4-sitl
```

The wrapper starts PX4 SITL with Gazebo inside the project container. The simulation container uses host networking, so it can reach the Pi's Ethernet address.

PX4 SITL starts its uXRCE-DDS client at `127.0.0.1:8888` by default. At the PX4 shell prompt, stop the default client and start it with the Pi address. Use `192.168.50.2` for Option A. For Option B, replace it with the Pi's reserved LAN address.

```text
uxrce_dds_client stop
uxrce_dds_client start -t udp -h 192.168.50.2 -p 8888
```

The `-h` option sets the Agent IP and `-p` sets the Agent UDP port. Keep `ROS_DOMAIN_ID` at `0` on the Pi and PX4 SITL unless both sides are configured with another matching domain ID. PX4 documents the client command and the default SITL address in the [PX4 v1.17 uXRCE-DDS guide](https://docs.px4.io/v1.17/en/middleware/uxrce_dds).

Do not use `./scripts/tmux-session` unchanged for this layout. That script also starts a local Agent on the desktop.

## Verify ROS 2 topics

In another Raspberry Pi terminal, source the same overlays and list PX4 topics:

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/px4_ros2/install/setup.bash"
source "$HOME/DroneSwarming/install/setup.bash"
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 topic list | grep '^/fmu/'
ros2 topic echo /fmu/out/vehicle_status_v1 --once
```

PX4 v1.17.0 exposes `/fmu/out/vehicle_status_v1`. The Agent terminal should report the PX4 client session and DDS entities when the connection succeeds.

## Keep the DDS graph local to the Pi

PX4 uXRCE-DDS topics have no DDS authentication. PX4 v1.17 defines `UXRCE_DDS_PTCFG=1` to restrict the Agent participant to localhost. Set `UXRCE_DDS_PTCFG` to `1` in QGroundControl's Parameters tool, then restart PX4 SITL. PX4 marks the parameter reboot-required.

After PX4 SITL restarts, run the `uxrce_dds_client stop` and `uxrce_dds_client start` commands from the previous section again. PX4 SITL starts its client at `127.0.0.1:8888` after restart.

Set `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` in each Pi ROS 2 terminal. This restricts ROS 2 discovery to the Pi while the uXRCE client connection continues over Ethernet UDP port `8888`.

See the [PX4 v1.17 parameter reference](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference.html#UXRCE_DDS_PTCFG) for the parameter values and the [uXRCE-DDS network guidance](https://docs.px4.io/v1.17/en/middleware/uxrce_dds#architecture).

Enable flight commands only in a controlled simulation after confirming PX4 telemetry, the system ID, and the recovery procedure. Use `enable_flight_commands:=true` when launching the application after those checks.
