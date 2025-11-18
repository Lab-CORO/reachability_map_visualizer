#!/usr/bin/env python3
"""
Script de détection de fuites mémoire pour reachability_map_visualizer
Utilise Valgrind pour analyser les binaires ROS 2
"""

import subprocess
import sys
import os
import argparse
from pathlib import Path
import re

class MemoryLeakDetector:
    def __init__(self, workspace_path="/home/user/reachability_map_visualizer"):
        self.workspace_path = Path(workspace_path)
        self.build_path = self.workspace_path / "build"

    def check_valgrind_installed(self):
        """Vérifie si Valgrind est installé"""
        try:
            result = subprocess.run(
                ["valgrind", "--version"],
                capture_output=True,
                text=True
            )
            print(f"✓ Valgrind trouvé: {result.stdout.strip()}")
            return True
        except FileNotFoundError:
            print("✗ Valgrind n'est pas installé. Installez-le avec:")
            print("  sudo apt-get install valgrind")
            return False

    def find_executables(self):
        """Trouve tous les exécutables dans le projet"""
        executables = []
        if self.build_path.exists():
            for file in self.build_path.rglob("*"):
                if file.is_file() and os.access(file, os.X_OK):
                    # Filtre pour ne garder que les binaires (pas les scripts)
                    if not file.suffix in ['.py', '.sh']:
                        executables.append(file)
        return executables

    def run_valgrind_analysis(self, executable, args=None, duration=10):
        """
        Exécute Valgrind sur un exécutable

        Args:
            executable: Chemin vers l'exécutable
            args: Arguments supplémentaires pour l'exécutable
            duration: Durée d'exécution en secondes
        """
        print(f"\n{'='*70}")
        print(f"Analyse de: {executable.name}")
        print(f"{'='*70}")

        cmd = [
            "valgrind",
            "--leak-check=full",
            "--show-leak-kinds=all",
            "--track-origins=yes",
            "--verbose",
            "--log-file=valgrind_output.txt",
            str(executable)
        ]

        if args:
            cmd.extend(args)

        try:
            # Lance le processus avec un timeout
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )

            try:
                stdout, stderr = process.communicate(timeout=duration)
            except subprocess.TimeoutExpired:
                process.kill()
                stdout, stderr = process.communicate()
                print(f"⚠ Analyse interrompue après {duration}s")

            # Lit le rapport Valgrind
            if os.path.exists("valgrind_output.txt"):
                with open("valgrind_output.txt", 'r') as f:
                    report = f.read()
                    self.parse_valgrind_report(report)

        except Exception as e:
            print(f"✗ Erreur lors de l'analyse: {e}")

    def parse_valgrind_report(self, report):
        """Parse et affiche les résultats Valgrind"""
        # Recherche des fuites mémoire
        leak_summary_match = re.search(
            r'LEAK SUMMARY:.*?(?=\n\n|\Z)',
            report,
            re.DOTALL
        )

        if leak_summary_match:
            print("\n📊 RÉSUMÉ DES FUITES:")
            print(leak_summary_match.group(0))

        # Compte le nombre de fuites
        definitely_lost = re.search(r'definitely lost: ([\d,]+) bytes', report)
        indirectly_lost = re.search(r'indirectly lost: ([\d,]+) bytes', report)
        possibly_lost = re.search(r'possibly lost: ([\d,]+) bytes', report)

        total_leaks = 0
        if definitely_lost:
            bytes_lost = int(definitely_lost.group(1).replace(',', ''))
            if bytes_lost > 0:
                print(f"⚠ Fuites définitives: {bytes_lost} bytes")
                total_leaks += bytes_lost

        if indirectly_lost:
            bytes_lost = int(indirectly_lost.group(1).replace(',', ''))
            if bytes_lost > 0:
                print(f"⚠ Fuites indirectes: {bytes_lost} bytes")
                total_leaks += bytes_lost

        if possibly_lost:
            bytes_lost = int(possibly_lost.group(1).replace(',', ''))
            if bytes_lost > 0:
                print(f"⚠ Fuites possibles: {bytes_lost} bytes")
                total_leaks += bytes_lost

        if total_leaks == 0:
            print("✓ Aucune fuite mémoire détectée!")
        else:
            print(f"\n⚠ TOTAL: {total_leaks} bytes de fuites potentielles")

    def analyze_code_static(self):
        """Analyse statique du code C++ pour détecter les problèmes potentiels"""
        print("\n" + "="*70)
        print("ANALYSE STATIQUE DU CODE")
        print("="*70)

        issues = []

        # Liste des fichiers à analyser
        cpp_files = list(self.workspace_path.glob("src/*.cpp"))
        h_files = list(self.workspace_path.glob("src/*.h"))
        h_files.extend(list(self.workspace_path.glob("include/**/*.h")))

        for file in cpp_files + h_files:
            with open(file, 'r') as f:
                content = f.read()
                lines = content.split('\n')

                for i, line in enumerate(lines, 1):
                    # Détecte les allocations avec new sans delete correspondant
                    if 'new ' in line and '=' in line:
                        var_name = self.extract_variable_name(line)
                        if var_name and not self.has_corresponding_delete(content, var_name):
                            issues.append({
                                'file': file.name,
                                'line': i,
                                'type': 'possible_leak',
                                'message': f"Allocation 'new' sans 'delete' visible pour '{var_name}'"
                            })

                    # Détecte les handles HDF5 non fermés
                    if re.search(r'H5[FGD]open', line):
                        handle_var = self.extract_variable_name(line)
                        if handle_var and not self.has_h5_close(content, handle_var):
                            issues.append({
                                'file': file.name,
                                'line': i,
                                'type': 'hdf5_leak',
                                'message': f"Handle HDF5 '{handle_var}' potentiellement non fermé"
                            })

                    # Détecte les variables globales/statiques de type shared_ptr
                    if re.search(r'(static|^)\s+.*shared_ptr', line) and '(' in line:
                        issues.append({
                            'file': file.name,
                            'line': i,
                            'type': 'static_shared_ptr',
                            'message': "shared_ptr global/statique qui ne sera jamais libéré"
                        })

        # Affiche les problèmes trouvés
        if issues:
            print(f"\n⚠ {len(issues)} problème(s) potentiel(s) détecté(s):\n")

            for issue in issues:
                icon = "🔴" if issue['type'] == 'possible_leak' else "🟡"
                print(f"{icon} {issue['file']}:{issue['line']}")
                print(f"   → {issue['message']}\n")
        else:
            print("✓ Aucun problème détecté dans l'analyse statique")

        return issues

    def extract_variable_name(self, line):
        """Extrait le nom de variable d'une ligne d'allocation"""
        # Patterns pour extraire le nom de variable
        patterns = [
            r'(\w+)\s*=\s*new\s+',
            r'(\w+)\s*=\s*H5[FGD]open',
        ]

        for pattern in patterns:
            match = re.search(pattern, line)
            if match:
                return match.group(1)
        return None

    def has_corresponding_delete(self, content, var_name):
        """Vérifie si une variable a un delete correspondant"""
        delete_patterns = [
            f'delete\\s+{var_name}',
            f'{var_name}\\s*=\\s*nullptr',
            f'reset\\s*\\(\\s*{var_name}\\s*\\)',
        ]

        for pattern in delete_patterns:
            if re.search(pattern, content):
                return True
        return False

    def has_h5_close(self, content, handle_var):
        """Vérifie si un handle HDF5 est fermé"""
        close_patterns = [
            f'H5[FGD]close\\s*\\(\\s*{handle_var}',
        ]

        for pattern in close_patterns:
            if re.search(pattern, content):
                return True
        return False

    def generate_report(self, output_file="memory_leak_report.md"):
        """Génère un rapport détaillé"""
        with open(output_file, 'w') as f:
            f.write("# Rapport de Détection de Fuites Mémoire\n\n")
            f.write(f"Projet: reachability_map_visualizer\n")
            f.write(f"Date: {subprocess.check_output(['date']).decode().strip()}\n\n")

            f.write("## Analyse Statique\n\n")
            issues = self.analyze_code_static()

            if issues:
                for issue in issues:
                    f.write(f"### {issue['file']}:{issue['line']}\n")
                    f.write(f"**Type:** {issue['type']}\n\n")
                    f.write(f"**Message:** {issue['message']}\n\n")
            else:
                f.write("Aucun problème détecté.\n\n")

        print(f"\n✓ Rapport généré: {output_file}")


def main():
    parser = argparse.ArgumentParser(
        description="Détection de fuites mémoire pour reachability_map_visualizer"
    )
    parser.add_argument(
        "--valgrind",
        action="store_true",
        help="Exécute l'analyse Valgrind (nécessite que le projet soit compilé)"
    )
    parser.add_argument(
        "--static",
        action="store_true",
        help="Exécute l'analyse statique du code"
    )
    parser.add_argument(
        "--report",
        action="store_true",
        help="Génère un rapport détaillé"
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Exécute toutes les analyses"
    )

    args = parser.parse_args()

    detector = MemoryLeakDetector()

    # Si aucun argument, affiche l'aide
    if not any([args.valgrind, args.static, args.report, args.all]):
        args.static = True  # Par défaut, analyse statique

    if args.all:
        args.valgrind = True
        args.static = True
        args.report = True

    print("🔍 Détecteur de Fuites Mémoire - reachability_map_visualizer")
    print("="*70)

    if args.static or args.report:
        detector.analyze_code_static()

    if args.valgrind:
        if detector.check_valgrind_installed():
            executables = detector.find_executables()
            if executables:
                print(f"\n✓ {len(executables)} exécutable(s) trouvé(s)")
                for exe in executables:
                    detector.run_valgrind_analysis(exe)
            else:
                print("\n⚠ Aucun exécutable trouvé. Compilez d'abord le projet.")

    if args.report:
        detector.generate_report()


if __name__ == "__main__":
    main()
