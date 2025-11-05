#!/bin/bash

# Script para organizar estrutura do pacote ROS2 Somanet
# Autor: Conversão ROS1 para ROS2
# Data: 2025-10-28

set -e

echo "=================================="
echo "Organizando pacote Somanet ROS2"
echo "=================================="
echo ""

# Obter diretório do script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

echo "📁 Diretório atual: $SCRIPT_DIR"
echo ""

# Criar estrutura de diretórios
echo "🔧 Criando estrutura de diretórios..."
mkdir -p src
mkdir -p scripts
mkdir -p launch
mkdir -p config

echo "✅ Diretórios criados"
echo ""

# Mover arquivos C++ para src/
echo "📦 Movendo arquivos C++ para src/..."
if [ -f "main.cpp" ]; then
    mv main.cpp src/
    echo "  ✓ main.cpp → src/"
fi

if [ -f "teleop_joy.cpp" ]; then
    mv teleop_joy.cpp src/
    echo "  ✓ teleop_joy.cpp → src/"
fi

echo ""

# Mover scripts Python para scripts/
echo "🐍 Movendo scripts Python para scripts/..."
if [ -f "teleop_keyboard.py" ]; then
    mv teleop_keyboard.py scripts/
    chmod +x scripts/teleop_keyboard.py
    echo "  ✓ teleop_keyboard.py → scripts/"
fi

echo ""

# Remover launch file antigo (ROS1)
echo "🗑️  Removendo arquivos ROS1..."
if [ -f "launch.launch" ]; then
    rm launch.launch
    echo "  ✓ Removido launch.launch (ROS1)"
fi

if [ -f "README_ROS2.md" ]; then
    rm README_ROS2.md
    echo "  ✓ Removido README_ROS2.md (duplicado)"
fi

echo ""

# Verificar se launch.py existe
if [ -f "launch/launch.py" ]; then
    echo "✅ launch/launch.py existe"
else
    echo "⚠️  launch/launch.py não encontrado"
fi

echo ""

# Resumo da estrutura
echo "📋 Estrutura final:"
echo ""
tree -L 2 -I '__pycache__|*.pyc' 2>/dev/null || {
    echo "├── CMakeLists.txt"
    echo "├── package.xml"
    echo "├── README.md"
    echo "├── QUICKSTART.md"
    echo "├── src/"
    echo "│   ├── main.cpp"
    echo "│   └── teleop_joy.cpp"
    echo "├── scripts/"
    echo "│   └── teleop_keyboard.py"
    echo "└── launch/"
    echo "    └── launch.py"
}

echo ""
echo "=================================="
echo "✅ Organização concluída!"
echo "=================================="
echo ""
echo "Próximos passos:"
echo "1. Compilar: colcon build --packages-select somanet"
echo "2. Source: source install/setup.bash"
echo "3. Executar: sudo -E env \"PATH=\$PATH\" ros2 launch somanet launch.py"
echo ""
