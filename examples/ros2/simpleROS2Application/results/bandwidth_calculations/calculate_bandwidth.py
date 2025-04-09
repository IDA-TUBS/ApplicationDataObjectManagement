from datetime import datetime
import sys

def calculate_bandwidth(logfile_path):
    total_sent_bytes = 0
    first_timestamp = None
    last_timestamp = None

    try:
        with open(logfile_path, 'r') as logfile:
            for line in logfile:
                # Zeilenverarbeitung
                if line.strip():  # Überprüfen, ob die Zeile nicht leer ist
                    try:
                        timestamp_str, sent_bytes_str, _ = line.split(',')
                        
                        # Parsing the timestamp
                        timestamp = datetime.strptime(timestamp_str, "%Y-%m-%d %H:%M:%S.%f")
                        
                        # Update first and last timestamps
                        if first_timestamp is None:
                            first_timestamp = timestamp
                        last_timestamp = timestamp
                        
                        # Extracting sent_bytes and summing them up
                        sent_bytes = int(sent_bytes_str.split(': ')[1])
                        total_sent_bytes += sent_bytes
                    except (ValueError, IndexError) as e:
                        print(f"Fehler beim Verarbeiten der Zeile: {line.strip()}. Fehler: {e}")

        # Berechnung der Zeitdifferenz
        if first_timestamp and last_timestamp:
            time_difference = (last_timestamp - first_timestamp).total_seconds()
        
            # Berechnung der Bandbreite
            if time_difference > 0:
                bandwidth = total_sent_bytes / time_difference
            else:
                bandwidth = 0

            # Ausgabe der Ergebnisse
            print(f"Total sent bytes: {total_sent_bytes}")
            print(f"Time difference (seconds): {time_difference}")
            print(f"Calculated bandwidth (B/s): {bandwidth}")
        else:
            print("Keine gültigen Zeitstempel gefunden.")

    except FileNotFoundError:
        print(f"Die Datei '{logfile_path}' wurde nicht gefunden.")
    except Exception as e:
        print(f"Ein Fehler ist aufgetreten: {e}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Verwendung: python script.py <path_to_logfile>")
    else:
        logfile_path = sys.argv[1]
        calculate_bandwidth(logfile_path)
