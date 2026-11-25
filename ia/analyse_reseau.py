# -*- coding: utf-8 -*-
"""
TAB-Géoloc – Apprentissage du niveau de danger (réseau de neurones)
===================================================================

Ce programme sert à 4 choses :

  0) ENVOYER un programme sur le Tab5 (trouve tout seul la carte et le port) :
        python analyse_reseau.py televerser Tab5_Collecte

  1) CAPTURER les mesures envoyées par le Tab5 (programme Tab5_Collecte) :
        python analyse_reseau.py capture AUTO AUTO
     (ou un port et un fichier précis : capture COM5 mesures/salle_B12.csv)

  2) ANALYSER le réseau de neurones : il essaie plusieurs réglages et plusieurs
     jeux d'entrées, puis compare les résultats :
        python analyse_reseau.py analyse
     -> crée le dossier "resultats/" (tableau + graphiques)
     -> crée "Tab5_IA/modele.h" : le meilleur réseau, prêt pour le Tab5

  3) Tester sans matériel avec des mesures SIMULÉES :
        python analyse_reseau.py demo
        python analyse_reseau.py analyse

Entrées du réseau (calculées par le Tab5) :
  det1, rssi1  : balise ZONE      -> détectée (1/0) et RSSI filtré (-100 si non détectée)
  det2, rssi2  : balise OBSTACLE  -> pareil
  heure        : facultative, codée en sinus/cosinus (23 h et 0 h sont proches)
  temperature  : facultative, capteur interne du Tab5

Sortie : niveau de danger  0 = LOIN, 1 = PROCHE, 2 = TRES PROCHE

Bibliothèques : pip install numpy pandas scikit-learn matplotlib pyserial
"""
import sys, os, glob, time, math
import numpy as np

# ------------------------------------------------------------------ RÉGLAGES
SEUIL_TRES_PROCHE = 1.0   # m : en dessous -> TRES PROCHE
SEUIL_PROCHE      = 2.5   # m : en dessous -> PROCHE, au-dessus -> LOIN
NIVEAUX = ["LOIN", "PROCHE", "TRES PROCHE"]

# Les jeux d'entrées comparés (le programme répond à : "faut-il ajouter l'heure ? la température ?")
JEUX_ENTREES = {
    "4 infos (det + RSSI)":     ["det1", "rssi1", "det2", "rssi2"],
    "4 infos + heure":          ["det1", "rssi1", "det2", "rssi2", "h_sin", "h_cos"],
    "4 infos + température":    ["det1", "rssi1", "det2", "rssi2", "temperature"],
    "4 infos + heure + temp.":  ["det1", "rssi1", "det2", "rssi2", "h_sin", "h_cos", "temperature"],
}
# Les architectures comparées : neurones par couche cachée
COUCHES = [(4,), (8,), (16,), (8, 8), (16, 8)]
ACTIVATIONS = ["relu", "tanh"]

DOSSIER_MESURES = "mesures"
DOSSIER_RESULTATS = "resultats"
FICHIER_MODELE = os.path.join("Tab5_IA", "modele.h")
COLONNES = ["session", "temps_ms", "distance_m", "det1", "rssi1", "det2", "rssi2", "heure", "temperature"]


# ================================================================== 0) OUTILS : captures d'écran, port USB du Tab5, téléversement
DOSSIER_CAPTURES = "captures"

def capture_ecran(nom):
    """Enregistre une capture de tout l'écran du PC dans captures/ (pour la présentation). Windows seulement."""
    if os.name != "nt":
        return
    import subprocess
    os.makedirs(DOSSIER_CAPTURES, exist_ok=True)
    chemin = os.path.abspath(os.path.join(DOSSIER_CAPTURES, "%s_%s.png" % (time.strftime("%Y%m%d_%H%M%S"), nom)))
    ps = ("Add-Type -AssemblyName System.Windows.Forms,System.Drawing;"
          "$b=[System.Windows.Forms.SystemInformation]::VirtualScreen;"
          "$i=New-Object System.Drawing.Bitmap $b.Width,$b.Height;"
          "$g=[System.Drawing.Graphics]::FromImage($i);"
          "$g.CopyFromScreen($b.Left,$b.Top,0,0,$i.Size);"
          "$i.Save('%s',[System.Drawing.Imaging.ImageFormat]::Png)" % chemin.replace("'", "''"))
    try:
        subprocess.run(["powershell", "-NoProfile", "-Command", ps], timeout=20, capture_output=True)
        print("  [capture d'écran] %s" % chemin)
    except Exception:
        pass

def trouver_port():
    """Cherche le port USB du Tab5 (puce Espressif, identifiant USB 303A)."""
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    for p in ports:
        if p.vid == 0x303A:          # seulement le Tab5 (les cartes des balises ont une autre puce USB)
            return p.device
    sys.exit("Tab5 introuvable. Ports vus : %s. Branche le Tab5 (câble USB-C données) et allume-le."
             % ([p.device for p in ports] or "aucun"))


def trouver_arduino_cli():
    import shutil
    candidats = [shutil.which("arduino-cli") or ""]
    for base in [os.environ.get("LOCALAPPDATA", ""), os.environ.get("ProgramFiles", ""), os.environ.get("ProgramFiles(x86)", "")]:
        candidats.append(os.path.join(base, "Programs", "Arduino IDE", "resources", "app", "lib", "backend", "resources", "arduino-cli.exe"))
        candidats.append(os.path.join(base, "Arduino IDE", "resources", "app", "lib", "backend", "resources", "arduino-cli.exe"))
    for c in candidats:
        if c and os.path.isfile(c):
            return c
    sys.exit("arduino-cli introuvable : Arduino IDE 2 est-il installé ?")


def televerser(dossier_sketch):
    """Compile et envoie un programme sur le Tab5 avec l'outil en ligne de commande d'Arduino IDE."""
    import json, subprocess
    cli = trouver_arduino_cli()
    base = [cli]
    conf = os.path.join(os.path.expanduser("~"), ".arduinoIDE", "arduino-cli.yaml")
    if os.path.isfile(conf):
        base += ["--config-file", conf]           # mêmes cartes et bibliothèques qu'Arduino IDE
    sortie = subprocess.run(base + ["board", "listall", "--format", "json"], capture_output=True, text=True).stdout
    donnees = json.loads(sortie or "{}")
    cartes = donnees.get("boards", donnees) if isinstance(donnees, dict) else donnees
    tab5 = [c for c in cartes if "tab5" in (c.get("name", "") + c.get("fqbn", "")).lower()]
    if not tab5:
        sys.exit("Carte Tab5 introuvable dans Arduino : installer le paquet de cartes M5Stack.")
    tab5.sort(key=lambda c: 0 if c["fqbn"].startswith("m5stack:") else 1)
    fqbn = tab5[0]["fqbn"]
    # active "USB CDC On Boot" si l'option existe, pour que les messages sortent par le câble USB
    details = subprocess.run(base + ["board", "details", "-b", fqbn, "--format", "json"], capture_output=True, text=True).stdout
    try:
        for opt in json.loads(details).get("config_options", []):
            if "cdc" in opt.get("option", "").lower():
                for v in opt.get("values", []):
                    if "enable" in v.get("value_label", "").lower():
                        fqbn += ":%s=%s" % (opt["option"], v["value"])
                        break
    except Exception:
        pass
    port = trouver_port()
    print("Carte : %s\nPort  : %s\nProgramme : %s\nCompilation + envoi (1 à 3 minutes)..." % (fqbn, port, dossier_sketch))
    # la sortie est affichée ET gardée dans un fichier journal (utile pour lire les erreurs)
    journal = "journal_%s.txt" % os.path.basename(os.path.normpath(dossier_sketch))
    with open(journal, "w", encoding="utf-8") as j:
        p = subprocess.Popen(base + ["compile", "--fqbn", fqbn, "--upload", "-p", port, os.path.abspath(dossier_sketch)],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding="utf-8", errors="replace")
        for ligne in p.stdout:
            print(ligne, end=""); j.write(ligne)
        p.wait()
    r = p
    print("\nTERMINÉ : programme envoyé sur le Tab5." if r.returncode == 0 else "\nÉCHEC : voir le message ci-dessus.")
    time.sleep(1)
    capture_ecran("televersement_%s_%s" % (os.path.basename(os.path.normpath(dossier_sketch)), "ok" if r.returncode == 0 else "echec"))
    sys.exit(r.returncode)


# ================================================================== 1) CAPTURE
def capture(port, fichier):
    import serial
    if port.upper() == "AUTO":
        port = trouver_port()
    if fichier.upper() == "AUTO":                 # nom de fichier automatique avec la date et l'heure
        fichier = os.path.join(DOSSIER_MESURES, "mesures_tab5.csv")   # toujours le même fichier : pas de doublons
    os.makedirs(os.path.dirname(fichier) or ".", exist_ok=True)
    print("Écoute de %s -> %s   (Ctrl+C pour arrêter)" % (port, fichier))
    nb = 0
    deja = set()                                   # lignes déjà enregistrées (évite les doublons)
    if os.path.isfile(fichier):
        for l in open(fichier, encoding="utf-8"):
            deja.add(";".join(l.strip().split(";")[:2]))
    with open(fichier, "a", encoding="utf-8") as f:
        if f.tell() == 0:
            f.write(";".join(COLONNES) + "\n"); f.flush()
        try:
            while True:
                # si le Tab5 redémarre ou est débranché, on attend qu'il revienne et on reprend
                try:
                    with serial.Serial(port, 115200, timeout=1) as s:
                        time.sleep(1.5)
                        s.write(b"ENVOI\n")                 # demande au Tab5 toute sa mémoire
                        print("  Récupération de la mémoire du Tab5...")
                        while True:
                            ligne = s.readline().decode("utf-8", "ignore").strip()
                            if ligne.startswith("M;"):      # seules les lignes de mesure commencent par "M;"
                                cle = ";".join(ligne[2:].split(";")[:2])   # session + temps
                                if cle in deja:
                                    continue
                                deja.add(cle)
                                f.write(ligne[2:] + "\n"); f.flush(); nb += 1
                                if nb % 25 == 0:
                                    print("  %d mesures enregistrées (dernière : %s)" % (nb, ligne[2:]))
                            elif ligne:
                                print("  [Tab5] " + ligne)
                                if ligne.startswith("# fin session"):
                                    capture_ecran("collecte_%d_mesures" % nb)
                except (serial.SerialException, OSError):
                    print("  Tab5 déconnecté (redémarrage ?) : reconnexion...")
                    time.sleep(2)
                    try:
                        port = trouver_port()
                    except SystemExit:
                        pass
        except KeyboardInterrupt:
            pass
    print("Terminé : %d mesures dans %s" % (nb, fichier))


# ================================================================== 2) DONNÉES SIMULÉES
def demo():
    """Crée des mesures réalistes : RSSI qui baisse avec la distance + bruit + petites
    différences d'une session à l'autre. L'heure et la température sont tirées au hasard."""
    rng = np.random.default_rng(1)
    positions = [0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0, 99.0]   # 99 = hors de portée
    lignes, session = [], 0
    for rep in range(4):
        for d in positions:
            session += 1
            decal1, decal2 = rng.normal(0, 2, 2)            # conditions qui changent un peu
            heure = rng.uniform(8, 18)
            temp = rng.uniform(19, 25)
            dz = abs(6.0 - min(d, 12)) + 0.5                  # balise ZONE placée à 6 m de l'obstacle
            for k in range(75):                               # 15 s à 5 mesures/s
                r2 = -56 - 25 * math.log10(min(d, 40)) + decal2 + rng.normal(0, 3)
                r1 = -60 - 25 * math.log10(dz) + decal1 + rng.normal(0, 3)
                det2 = 1 if (d < 40 and r2 > -92 and rng.random() > 0.03) else 0
                det1 = 1 if (r1 > -92 and rng.random() > 0.03) else 0
                lignes.append([session, k * 200, d, det1, round(r1, 1) if det1 else -100.0,
                               det2, round(r2, 1) if det2 else -100.0, round(heure + k * 0.2 / 3600, 4), round(temp, 1)])
    os.makedirs(DOSSIER_MESURES, exist_ok=True)
    chemin = os.path.join(DOSSIER_MESURES, "demo_simulee.csv")
    with open(chemin, "w", encoding="utf-8") as f:
        f.write(";".join(COLONNES) + "\n")
        for l in lignes:
            f.write(";".join(str(x) for x in l) + "\n")
    print("Mesures simulées : %d lignes dans %s" % (len(lignes), chemin))
    print("ATTENTION : supprimer ce fichier avant d'analyser les vraies mesures.")


# ================================================================== 3) ANALYSE
def charger():
    import pandas as pd
    fichiers = sorted(glob.glob(os.path.join(DOSSIER_MESURES, "*.csv")))
    if not fichiers:
        sys.exit("Aucun fichier .csv dans le dossier '%s'." % DOSSIER_MESURES)
    tables = []
    for i, f in enumerate(fichiers):
        t = pd.read_csv(f, sep=";")
        t["groupe"] = t["session"].astype(str) + "_" + str(i)   # une session = un enregistrement de 15 s
        tables.append(t)
        print("  %-35s %5d mesures" % (os.path.basename(f), len(t)))
    df = pd.concat(tables, ignore_index=True)
    df["h_sin"] = np.sin(2 * np.pi * df["heure"] / 24)
    df["h_cos"] = np.cos(2 * np.pi * df["heure"] / 24)
    df["niveau"] = np.where(df["distance_m"] < SEUIL_TRES_PROCHE, 2,
                   np.where(df["distance_m"] < SEUIL_PROCHE, 1, 0))
    return df


def nb_parametres(n_entrees, couches, n_sorties=3):
    tailles = [n_entrees] + list(couches) + [n_sorties]
    return sum(tailles[i] * tailles[i + 1] + tailles[i + 1] for i in range(len(tailles) - 1))


def analyse():
    import pandas as pd
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from sklearn.pipeline import make_pipeline
    from sklearn.preprocessing import StandardScaler
    from sklearn.neural_network import MLPClassifier
    from sklearn.model_selection import StratifiedGroupKFold, cross_val_score, cross_val_predict
    from sklearn.metrics import confusion_matrix, ConfusionMatrixDisplay, accuracy_score
    import warnings
    warnings.filterwarnings("ignore")

    print("Chargement des mesures :")
    df = charger()
    y = df["niveau"].values
    groupes = df["groupe"].values
    print("Répartition : " + ", ".join("%s = %d" % (NIVEAUX[k], (y == k).sum()) for k in range(3)))
    n_groupes = len(set(groupes))
    k = min(5, n_groupes)
    if k < 3:
        sys.exit("Pas assez d'enregistrements (il en faut au moins 3, idéalement 5 par niveau).")
    # On teste sur des ENREGISTREMENTS que le réseau n'a jamais vus (pas juste des lignes au hasard),
    # sinon le score serait trop optimiste.
    cv = StratifiedGroupKFold(n_splits=k, shuffle=True, random_state=0)

    def reseau(couches, activation):
        return make_pipeline(StandardScaler(),
                             MLPClassifier(hidden_layer_sizes=couches, activation=activation, alpha=1e-3,
                                           learning_rate_init=0.01, max_iter=1500, random_state=0))

    resultats = []
    print("\nComparaison des configurations (précision = %% de bonnes décisions, validation croisée %d plis) :" % k)
    for nom_jeu, cols in JEUX_ENTREES.items():
        X = df[cols].values
        for couches in COUCHES:
            for act in ACTIVATIONS:
                t0 = time.time()
                scores = cross_val_score(reseau(couches, act), X, y, groups=groupes, cv=cv)
                resultats.append({"entrees": nom_jeu, "couches": str(couches), "activation": act,
                                  "precision_moy": scores.mean(), "precision_ecart": scores.std(),
                                  "nb_parametres": nb_parametres(len(cols), couches), "duree_s": time.time() - t0})
                print("  %-26s %-8s %-5s : %5.1f %% (± %.1f)  %4d paramètres"
                      % (nom_jeu, str(couches), act, 100 * scores.mean(), 100 * scores.std(),
                         nb_parametres(len(cols), couches)))
    res = pd.DataFrame(resultats)
    os.makedirs(DOSSIER_RESULTATS, exist_ok=True)
    res.sort_values("precision_moy", ascending=False).to_csv(
        os.path.join(DOSSIER_RESULTATS, "comparaison.csv"), sep=";", index=False, decimal=",")

    # ---- choix : meilleure précision ; à 1 point près, on préfère le réseau le plus petit
    meilleur_score = res["precision_moy"].max()
    candidats = res[res["precision_moy"] >= meilleur_score - 0.01].sort_values(["nb_parametres", "precision_moy"],
                                                                                  ascending=[True, False])
    choix = candidats.iloc[0]
    cols = JEUX_ENTREES[choix["entrees"]]
    couches = eval(choix["couches"])
    act = choix["activation"]

    # ---- est-ce que l'heure / la température aident ?
    base = res[res["entrees"] == "4 infos (det + RSSI)"]["precision_moy"].max()
    print("\nApport des paramètres en plus (meilleur score de chaque jeu d'entrées) :")
    for nom_jeu in JEUX_ENTREES:
        s = res[res["entrees"] == nom_jeu]["precision_moy"].max()
        print("  %-26s %5.1f %%   (%+.1f point par rapport aux 4 infos)" % (nom_jeu, 100 * s, 100 * (s - base)))

    # ---- graphique de comparaison
    fig, ax = plt.subplots(figsize=(11, 5))
    etiquettes = [c + " " + a for c in map(str, COUCHES) for a in ACTIVATIONS]
    x = np.arange(len(etiquettes))
    largeur = 0.8 / len(JEUX_ENTREES)
    for j, nom_jeu in enumerate(JEUX_ENTREES):
        sous = res[res["entrees"] == nom_jeu]
        valeurs = [100 * sous[(sous["couches"] == str(c)) & (sous["activation"] == a)]["precision_moy"].iloc[0]
                   for c in COUCHES for a in ACTIVATIONS]
        ax.bar(x + j * largeur - 0.4 + largeur / 2, valeurs, largeur, label=nom_jeu)
    ax.set_xticks(x); ax.set_xticklabels(etiquettes, rotation=30, ha="right")
    ax.set_ylabel("Précision (%)"); ax.set_ylim(max(0, 100 * res["precision_moy"].min() - 10), 100)
    ax.set_title("Comparaison des réseaux de neurones (neurones par couche + activation)")
    ax.legend(fontsize=8); ax.grid(axis="y", alpha=0.3)
    fig.tight_layout(); fig.savefig(os.path.join(DOSSIER_RESULTATS, "comparaison.png"), dpi=130); plt.close(fig)

    # ---- matrice de confusion du réseau choisi (sur des enregistrements jamais vus)
    X = df[cols].values
    pred = cross_val_predict(reseau(couches, act), X, y, groups=groupes, cv=cv)
    fig, ax = plt.subplots(figsize=(5, 4.5))
    ConfusionMatrixDisplay(confusion_matrix(y, pred, labels=[0, 1, 2]), display_labels=NIVEAUX).plot(ax=ax, cmap="Blues", colorbar=False)
    ax.set_xlabel("Décision du réseau"); ax.set_ylabel("Vrai niveau")
    ax.set_title("Réseau choisi : %.1f %% de bonnes décisions" % (100 * accuracy_score(y, pred)))
    fig.tight_layout(); fig.savefig(os.path.join(DOSSIER_RESULTATS, "matrice_confusion.png"), dpi=130); plt.close(fig)

    # ---- entraînement final sur TOUTES les mesures + courbe d'apprentissage
    modele = reseau(couches, act).fit(X, y)
    mlp, norm = modele[-1], modele[0]
    fig, ax = plt.subplots(figsize=(6, 4))
    ax.plot(mlp.loss_curve_); ax.set_xlabel("Itération"); ax.set_ylabel("Erreur (log-loss)")
    ax.set_title("Apprentissage du réseau choisi"); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(os.path.join(DOSSIER_RESULTATS, "courbe_apprentissage.png"), dpi=130); plt.close(fig)

    exporter(norm, mlp, cols, act, choix)
    print("\nRéseau choisi : entrées = %s | couches cachées = %s | activation = %s | %d paramètres"
          % (choix["entrees"], couches, act, choix["nb_parametres"]))
    print("Précision sur des enregistrements jamais vus : %.1f %% (± %.1f)" % (100 * choix["precision_moy"], 100 * choix["precision_ecart"]))
    print("Fichiers créés : %s/comparaison.csv, comparaison.png, matrice_confusion.png, courbe_apprentissage.png" % DOSSIER_RESULTATS)
    print("                 %s  (déjà à côté de Tab5_IA.ino)" % FICHIER_MODELE)
    time.sleep(0.5)
    capture_ecran("analyse_reseau_resultats")


def tableau_c(nom, valeurs):
    v = ", ".join("%.7gf" % x for x in np.ravel(valeurs))
    return "const float %s[%d] = {%s};\n" % (nom, np.size(valeurs), v)


def exporter(norm, mlp, cols, act, choix):
    tailles = [len(cols)] + [w.shape[1] for w in mlp.coefs_]
    lignes = ["// modele.h – réseau de neurones généré par analyse_reseau.py le %s" % time.strftime("%d/%m/%Y %H:%M"),
              "// Entrées : %s" % ", ".join(cols),
              "// Couches : %s | activation : %s | précision estimée : %.1f %%" % (tailles, act, 100 * choix["precision_moy"]),
              "// Niveaux : 0 = LOIN (>= %.1f m), 1 = PROCHE, 2 = TRES PROCHE (< %.1f m)" % (SEUIL_PROCHE, SEUIL_TRES_PROCHE),
              "#pragma once", ""]
    lignes.append("const int N_ENTREES = %d;" % len(cols))
    lignes.append("const int N_COUCHES = %d;   // nombre de calculs (couches cachées + sortie)" % len(mlp.coefs_))
    lignes.append("const int TAILLES[%d] = {%s};" % (len(tailles), ", ".join(map(str, tailles))))
    lignes.append("const bool ACTIVATION_RELU = %s;   // sinon tanh" % ("true" if act == "relu" else "false"))
    lignes.append("const bool UTILISE_HEURE = %s;" % ("true" if "h_sin" in cols else "false"))
    lignes.append("const bool UTILISE_TEMPERATURE = %s;" % ("true" if "temperature" in cols else "false"))
    lignes.append("const float SEUIL_TRES_PROCHE_M = %.2ff, SEUIL_PROCHE_M = %.2ff;" % (SEUIL_TRES_PROCHE, SEUIL_PROCHE))
    lignes.append("// normalisation : x_norm = (x - MOYENNE) / ECART")
    texte = "\n".join(lignes) + "\n"
    texte += tableau_c("MOYENNE", norm.mean_) + tableau_c("ECART", norm.scale_)
    for i, (w, b) in enumerate(zip(mlp.coefs_, mlp.intercepts_)):
        texte += "// couche %d : %d -> %d  (poids rangés [entrée][sortie])\n" % (i + 1, w.shape[0], w.shape[1])
        texte += tableau_c("W%d" % i, w) + tableau_c("B%d" % i, b)
    texte += "const float* const POIDS[%d] = {%s};\n" % (len(mlp.coefs_), ", ".join("W%d" % i for i in range(len(mlp.coefs_))))
    texte += "const float* const BIAIS[%d] = {%s};\n" % (len(mlp.coefs_), ", ".join("B%d" % i for i in range(len(mlp.coefs_))))
    os.makedirs(os.path.dirname(FICHIER_MODELE), exist_ok=True)
    with open(FICHIER_MODELE, "w", encoding="utf-8") as f:
        f.write(texte)


# ================================================================== MAIN
if __name__ == "__main__":
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    if len(sys.argv) >= 3 and sys.argv[1] == "televerser":
        televerser(sys.argv[2])
    elif len(sys.argv) >= 4 and sys.argv[1] == "capture":
        capture(sys.argv[2], sys.argv[3])
    elif len(sys.argv) >= 2 and sys.argv[1] == "demo":
        demo()
    elif len(sys.argv) >= 2 and sys.argv[1] == "analyse":
        analyse()
    else:
        print(__doc__)
