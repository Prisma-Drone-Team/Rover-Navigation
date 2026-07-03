#!/usr/bin/env python3
"""
plot_energy.py — disegna Fig. 5.7: energia integrata vs distanza percorsa,
per la run "interrupt" e la run "controllo".

Uso:
    python3 plot_energy.py energy_interrupt.csv energy_control.csv [budget]

Produce val_energy_curve.pdf e .png nella cartella corrente.
"""
import sys, csv
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def load(path):
    dist, energy = [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            dist.append(float(row['distance']))
            energy.append(float(row['energy']))
    return dist, energy


def main():
    interrupt = sys.argv[1] if len(sys.argv) > 1 else 'energy_interrupt.csv'
    control = sys.argv[2] if len(sys.argv) > 2 else 'energy_control.csv'
    budget = float(sys.argv[3]) if len(sys.argv) > 3 else 40.0

    di, ei = load(interrupt)
    dc, ec = load(control)

    plt.figure(figsize=(6, 4))
    plt.plot(dc, ec, color='tab:blue', label='control run (high budget)')
    plt.plot(di, ei, color='tab:red', label='interrupt run (budget = %g)' % budget)
    plt.axhline(budget, ls='--', color='gray', lw=1.0, label='energy budget')

    # segna il punto in cui scatta battery_low (prima volta E >= budget)
    for k, val in enumerate(ei):
        if val >= budget:
            plt.scatter([di[k]], [ei[k]], color='tab:red', zorder=5)
            plt.annotate('battery\\_low $\\rightarrow$ return',
                         (di[k], ei[k]), textcoords='offset points',
                         xytext=(6, -14), fontsize=8)
            break

    plt.xlabel('travelled distance [m]')
    plt.ylabel('integrated energy $E$')
    plt.legend(loc='upper left', fontsize=9)
    plt.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig('val_energy_curve.pdf')
    plt.savefig('val_energy_curve.png', dpi=150)
    print('scritti: val_energy_curve.pdf / .png')


if __name__ == '__main__':
    main()
