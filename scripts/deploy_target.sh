#!/bin/bash
# scripts/deploy_target.sh

set -e

TARGET_IP="192.168.1.100"
TARGET_USER="root"
TARGET_DIR="/opt/vcu-system"
CONFIG_DIR="/etc/vcu"

echo "Deploying VCU System to target..."
echo "Target: ${TARGET_USER}@${TARGET_IP}"
echo "=========================================="

# 检查目标连接
echo "Checking target connection..."
ssh ${TARGET_USER}@${TARGET_IP} "echo 'Connection successful'"

# 创建目标目录
echo "Creating target directories..."
ssh ${TARGET_USER}@${TARGET_IP} "mkdir -p ${TARGET_DIR} ${CONFIG_DIR}"

# 传输可执行文件
echo "Deploying binaries..."
scp build/vcu_system ${TARGET_USER}@${TARGET_IP}:${TARGET_DIR}/

# 传输配置文件
echo "Deploying configuration..."
scp -r config/* ${TARGET_USER}@${TARGET_IP}:${CONFIG_DIR}/

# 传输脚本
echo "Deploying scripts..."
scp -r scripts/* ${TARGET_USER}@${TARGET_IP}:${TARGET_DIR}/scripts/

# 设置权限
echo "Setting permissions..."
ssh ${TARGET_USER}@${TARGET_IP} "chmod +x ${TARGET_DIR}/vcu_system"
ssh ${TARGET_USER}@${TARGET_IP} "chmod +x ${TARGET_DIR}/scripts/*.sh"

# 创建系统服务
echo "Creating system service..."
cat > vcu-system.service << EOF
[Unit]
Description=Autonomous Tractor VCU System
After=network.target

[Service]
Type=simple
User=root
WorkingDirectory=${TARGET_DIR}
ExecStart=${TARGET_DIR}/vcu_system --config ${CONFIG_DIR}
Restart=always
RestartSec=5
TimeoutStopSec=10

[Install]
WantedBy=multi-user.target
EOF

scp vcu-system.service ${TARGET_USER}@${TARGET_IP}:/etc/systemd/system/
rm vcu-system.service

# 启用并启动服务
echo "Starting VCU system service..."
ssh ${TARGET_USER}@${TARGET_IP} "
    systemctl daemon-reload
    systemctl enable vcu-system.service
    systemctl start vcu-system.service
    systemctl status vcu-system.service
"

echo "Deployment completed successfully!"
echo "System is running on ${TARGET_IP}"