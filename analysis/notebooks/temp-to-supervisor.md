Hi Lukas, hier die Ergebnisse zum Radar-Timing für die Odometry-Pipeline, bevor ich mich der Weihnachtszeit widme :D

## Methodik
**Velocity-Estimation:** Weighted Least Squares über Doppler-Punktwolke: $\mathbf{v}_{body} = \arg\min \sum w_i (\hat{r}_i \cdot \vec{v} - v_{rad,i})^2$ mit intensity-basierten Gewichten. Validierung gegen IMU-Integration (highpass 0.2 Hz, bias-korrigiert). Datensatz: 15s dynamisches Flugmanöver.

## 1. USB-Timing Problem
ROS-Timestamps nach USB-Übertragung: Jitter σ=7.15 ms, systematische Latenz 13.09 ms, worst-case bis 50 ms.

## 2. CPU-Cycle Count Lösung
Verwendung des Hardware-internen 200 MHz Counter. Automatische Overflow-Korrektur (= Counter Reset). Direkte Optimierung gegen IMU, nicht fehlerhafte ROS Timestamps: `t_radar = slope × cpu_cycles + intercept`.

**Ergebnis:** Theoretisch quasi kein Jitter/STD, Korrelation 0.9529 → 0.9545, RMS 0.233 → 0.230 m/s.

## 3. Vergleich zu TI-Filterung
Weitere 0,01% Verbesserung der Korrelation, vernachlässigbar.

## 4. Interpretation 
Eine bessere Korrelation bekomme ich mit den Daten glaube nicht hin. Die verbleibenden 4,5% Varianz sind vermutlich aufgrund des Flugprofils und sensor-intrinsisch (Doppler-Rauschen, Montage, Multipath, kein Plan was es noch gibt). 

Kannst du hieraus bereits was zur Feasability der Odometry ableiten? Ich wünsche dir noch ein Frohes Fest und schöne Feiertage!