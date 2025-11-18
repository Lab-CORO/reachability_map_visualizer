# Script de Détection de Fuites Mémoire

## Description

Ce dossier contient des outils pour détecter et analyser les fuites mémoire dans le projet `reachability_map_visualizer`.

## check_memory_leaks.py

Script Python pour détecter les fuites mémoire via analyse statique et dynamique.

### Fonctionnalités

1. **Analyse Statique** : Analyse le code C++ pour détecter les patterns suspects
   - Allocations `new` sans `delete` correspondant
   - Handles HDF5 non fermés
   - Variables globales/statiques de type `shared_ptr`

2. **Analyse Dynamique avec Valgrind** (optionnel) : Exécute les binaires avec Valgrind pour détecter les fuites à l'exécution

3. **Génération de Rapport** : Crée un rapport détaillé en Markdown

### Prérequis

```bash
# Pour l'analyse statique (toujours disponible)
python3

# Pour l'analyse avec Valgrind (optionnel)
sudo apt-get install valgrind
```

### Utilisation

```bash
# Rendre le script exécutable
chmod +x scripts/check_memory_leaks.py

# Analyse statique uniquement (recommandé)
python3 scripts/check_memory_leaks.py --static

# Analyse avec Valgrind (nécessite que le projet soit compilé)
python3 scripts/check_memory_leaks.py --valgrind

# Générer un rapport détaillé
python3 scripts/check_memory_leaks.py --report

# Toutes les analyses
python3 scripts/check_memory_leaks.py --all
```

### Interprétation des Résultats

#### Icônes
- 🔴 **Rouge** : Allocation `new` sans `delete` visible
- 🟡 **Jaune** : Handle HDF5 ou autre ressource potentiellement non fermée
- ✅ **Vert** : Aucun problème détecté

#### Types de Problèmes

1. **possible_leak** : Allocation avec `new` sans `delete` correspondant
   - ⚠️ Peut être un faux positif si la mémoire est gérée par un parent (cas des propriétés RViz)
   - ⚠️ Vérifier la documentation de la bibliothèque utilisée

2. **hdf5_leak** : Handle HDF5 potentiellement non fermé
   - ⚠️ Peut être un faux positif si fermé dans un destructeur
   - ✅ Toujours vérifier que `H5Fclose`, `H5Gclose`, `H5Dclose` sont appelés

3. **static_shared_ptr** : Pointeur partagé global/statique
   - ⚠️ DANGER : Ne sera jamais libéré
   - ⚠️ Si la structure accumule des données, cela causera une fuite progressive

### Exemple de Sortie

```
🔍 Détecteur de Fuites Mémoire - reachability_map_visualizer
======================================================================

======================================================================
ANALYSE STATIQUE DU CODE
======================================================================

⚠ 5 problème(s) potentiel(s) détecté(s):

🔴 example.cpp:42
   → Allocation 'new' sans 'delete' visible pour 'my_object'

🟡 hdf5_example.cpp:15
   → Handle HDF5 'file_handle' potentiellement non fermé
```

### Faux Positifs Connus

Le script d'analyse statique peut détecter certains faux positifs :

1. **Propriétés RViz** : Les propriétés créées avec `new` et passées à RViz sont gérées automatiquement par le système de propriétés parent-enfant de RViz2.

2. **Handles HDF5 fermés dans destructeur** : Le script ne peut pas toujours détecter les fermetures dans les destructeurs.

3. **Smart Pointers** : Les `std::unique_ptr` et `std::shared_ptr` gèrent automatiquement la mémoire.

### Conseils

- Exécutez l'analyse statique régulièrement lors du développement
- Utilisez Valgrind pour les tests d'intégration
- Consultez `MEMORY_LEAK_FIXES.md` pour voir les corrections déjà appliquées
- En cas de doute, préférez les smart pointers aux pointeurs bruts

### Limitations

- L'analyse statique ne peut pas détecter toutes les fuites complexes
- Valgrind nécessite que le code soit compilé avec les symboles de débogage
- Certaines bibliothèques (comme Qt, Ogre) peuvent rapporter des faux positifs

### Support

Pour plus d'informations sur les fuites mémoire déjà corrigées, voir `MEMORY_LEAK_FIXES.md` à la racine du projet.
