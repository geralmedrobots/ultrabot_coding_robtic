# SROS2 Quick Reference Card

## 🚀 Quick Start (5 minutes)

### 1. Generate Security Keystore
```bash
cd ~/ultrabot_ws/src/navigation/scripts
./setup_sros2.sh
```

### 2. Enable SROS2
```bash
export ROS_SECURITY_KEYSTORE=~/ultrabot_ws/src/navigation/sros2_keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
```

### 3. Launch
```bash
ros2 launch somanet launch.py
```

---

## 🔍 Verification Commands

```bash
# List secured nodes
ros2 security list_enclaves $ROS_SECURITY_KEYSTORE

# Check node security status
ros2 node list
ros2 node info /safety_supervisor

# Monitor security events
ros2 topic echo /rosout | grep -i security
```

---

## ⚙️ Environment Variables

| Variable | Values | Description |
|----------|--------|-------------|
| `ROS_SECURITY_ENABLE` | `true`/`false` | Enable/disable SROS2 |
| `ROS_SECURITY_KEYSTORE` | `/path/to/keystore` | Location of certificates |
| `ROS_SECURITY_STRATEGY` | `Enforce`/`Permissive` | Strict or relaxed mode |

**Modes:**
- `Enforce`: Deny all unauthorized access (production)
- `Permissive`: Allow but log violations (testing)

---

## 🛠️ Common Tasks

### Add New Node
```bash
# 1. Create enclave
ros2 security create_enclave $ROS_SECURITY_KEYSTORE /somanet/new_node

# 2. Create policy XML
nano config/security_policies/new_node.xml

# 3. Apply policy
ros2 security create_permission \
  $ROS_SECURITY_KEYSTORE \
  /somanet/new_node \
  config/security_policies/new_node.xml
```

### Rotate Certificates
```bash
# Backup old keystore
cp -r sros2_keystore sros2_keystore.backup

# Generate new keystore
./setup_sros2.sh

# Test before deploying
export ROS_SECURITY_KEYSTORE=~/ultrabot_ws/src/navigation/sros2_keystore
ros2 launch somanet launch.py
```

### Disable SROS2 (Emergency)
```bash
unset ROS_SECURITY_ENABLE
unset ROS_SECURITY_KEYSTORE
unset ROS_SECURITY_STRATEGY

ros2 launch somanet launch.py
```

---

## 🐛 Troubleshooting

| Issue | Solution |
|-------|----------|
| **"Authentication failed"** | Check `$ROS_SECURITY_KEYSTORE` path |
| **"Permission denied"** | Update policy XML, reapply with `create_permission` |
| **Nodes can't see each other** | Verify both nodes have enclaves in keystore |
| **High latency** | Normal (5-10% overhead), optimize if needed |

---

## 📊 Security Status Check

```bash
# Full security audit
echo "=== SROS2 Status ==="
echo "Enabled: $ROS_SECURITY_ENABLE"
echo "Keystore: $ROS_SECURITY_KEYSTORE"
echo "Strategy: $ROS_SECURITY_STRATEGY"
echo ""
echo "=== Enclaves ==="
ros2 security list_enclaves $ROS_SECURITY_KEYSTORE
echo ""
echo "=== Running Nodes ==="
ros2 node list
echo ""
echo "=== Recent Security Events ==="
ros2 topic echo /rosout --once | grep -i security
```

---

## 🔒 Production Checklist

- [ ] SROS2 enabled (`ROS_SECURITY_ENABLE=true`)
- [ ] Strategy set to `Enforce`
- [ ] Keystore permissions: `chmod 700 sros2_keystore`
- [ ] Private keys protected: `chmod 600 */key.pem`
- [ ] Keystore NOT in version control
- [ ] All nodes have enclaves
- [ ] All policies reviewed and minimal (principle of least privilege)
- [ ] Certificate expiration date documented
- [ ] Backup keystore stored securely
- [ ] Incident response plan documented

---

## 📞 Help

**Documentation:** [SROS2_GUIDE.md](SROS2_GUIDE.md)  
**ROS 2 Docs:** https://docs.ros.org/en/humble/Tutorials/Advanced/Security/

**Security Incident:** Stop robot, revoke certificates, audit logs
