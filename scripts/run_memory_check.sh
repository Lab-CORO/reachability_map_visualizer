#!/bin/bash
# Script simple pour vérifier les fuites mémoire
# Usage: ./scripts/run_memory_check.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "================================================"
echo "  Vérification des Fuites Mémoire"
echo "  reachability_map_visualizer"
echo "================================================"
echo ""

cd "$PROJECT_ROOT"

# Vérifier que Python est installé
if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 n'est pas installé"
    exit 1
fi

echo "✅ Python 3 trouvé"
echo ""

# Exécuter l'analyse statique
echo "🔍 Exécution de l'analyse statique..."
echo ""
python3 scripts/check_memory_leaks.py --static

echo ""
echo "================================================"
echo "  Analyse terminée!"
echo "================================================"
echo ""
echo "Pour plus d'informations:"
echo "  - Voir MEMORY_LEAK_FIXES.md pour les corrections"
echo "  - Voir scripts/README.md pour la documentation"
echo ""
echo "Analyses avancées:"
echo "  - Valgrind: python3 scripts/check_memory_leaks.py --valgrind"
echo "  - Rapport:  python3 scripts/check_memory_leaks.py --report"
echo ""
