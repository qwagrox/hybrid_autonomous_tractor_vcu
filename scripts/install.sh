#!/bin/bash
# VCU Service Installation Script

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Print functions
print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running as root
if [[ $EUID -ne 0 ]]; then
   print_error "This script must be run as root"
   exit 1
fi

print_info "Installing VCU Service..."

# Create VCU user and group
if ! id "vcu" &>/dev/null; then
    print_info "Creating VCU user and group..."
    useradd -r -s /bin/false -d /var/lib/vcu vcu
    usermod -a -G dialout vcu  # Add to dialout group for CAN access
else
    print_info "VCU user already exists"
fi

# Create directories
print_info "Creating directories..."
mkdir -p /etc/vcu
mkdir -p /var/log/vcu
mkdir -p /var/lib/vcu
mkdir -p /usr/local/bin

# Set permissions
chown vcu:vcu /var/log/vcu
chown vcu:vcu /var/lib/vcu
chmod 755 /etc/vcu
chmod 755 /var/log/vcu
chmod 755 /var/lib/vcu

# Install binary
print_info "Installing VCU main binary..."
if [ -f "build/vcu_main" ]; then
    cp build/vcu_main /usr/local/bin/
    chmod 755 /usr/local/bin/vcu_main
else
    print_error "VCU main binary not found. Please build the project first."
    exit 1
fi

# Install configuration
print_info "Installing configuration..."
if [ -f "config/vcu_config.json" ]; then
    cp config/vcu_config.json /etc/vcu/
    chmod 644 /etc/vcu/vcu_config.json
else
    print_warn "Default configuration not found, creating minimal config..."
    cat > /etc/vcu/vcu_config.json << EOF
{
  "can_bus_name": "can0",
  "data_timeout_ms": 1000,
  "system": {
    "main_loop_frequency_hz": 100,
    "diagnostic_level": "INFO"
  }
}
EOF
fi

# Install systemd service
print_info "Installing systemd service..."
if [ -f "scripts/vcu.service" ]; then
    cp scripts/vcu.service /etc/systemd/system/
    chmod 644 /etc/systemd/system/vcu.service
    systemctl daemon-reload
else
    print_error "Systemd service file not found"
    exit 1
fi

# Setup CAN interface (if available)
print_info "Setting up CAN interface..."
if command -v ip &> /dev/null; then
    # Create CAN interface setup script
    cat > /usr/local/bin/setup-can.sh << 'EOF'
#!/bin/bash
# Setup CAN interface for VCU

# Load CAN modules
modprobe can
modprobe can_raw
modprobe vcan

# Create virtual CAN interface for testing
if ! ip link show vcan0 &>/dev/null; then
    ip link add dev vcan0 type vcan
    ip link set up vcan0
fi

# Setup real CAN interface (uncomment and modify as needed)
# ip link set can0 type can bitrate 500000
# ip link set up can0
EOF
    chmod +x /usr/local/bin/setup-can.sh
    print_info "CAN setup script created at /usr/local/bin/setup-can.sh"
else
    print_warn "ip command not found, skipping CAN setup"
fi

# Enable and start service
print_info "Enabling VCU service..."
systemctl enable vcu.service

print_info "VCU Service installation completed successfully!"
print_info ""
print_info "Next steps:"
print_info "1. Review configuration in /etc/vcu/vcu_config.json"
print_info "2. Setup CAN interface: sudo /usr/local/bin/setup-can.sh"
print_info "3. Start the service: sudo systemctl start vcu"
print_info "4. Check status: sudo systemctl status vcu"
print_info "5. View logs: sudo journalctl -u vcu -f"
