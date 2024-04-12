import random
from collections import deque

# Zdefiniuj pulę kolorów z odpowiednią liczbą każdego koloru
kolory = [
    " 0 0 255\n",  # Jasnoniebieski - 1 raz
    " 255 0 0\n",  # Czerwony - 1 raz
    " 0 0 0\n",    # Czarny - 1 raz
] + [" 0 255 0\n"] * 6  # Jasny zielony - 6 razy

# Lista do przechowywania zmodyfikowanych linii
zmodyfikowane_linie = []

# Otwórz plik tekstowy do odczytu
with open('mars_mines.wbt', 'r') as file:
    lines = file.readlines()

# Lista do przechowywania indeksów linii z 'baseColor'
base_color_indices = []

# Przejście przez linie i zapisanie indeksów linii z 'baseColor'
for i, line in enumerate(lines):
    if "baseColor" in line:
        base_color_indices.append(i)

# Przejście przez linie ponownie, tym razem modyfikując te z 'baseColor', dla których warunek jest spełniony
for i, line in enumerate(lines):
    if i in base_color_indices:
        # Sprawdź, czy 13 linii później jest 'name "ball"'
        if i + 13 < len(lines) and 'name "ball' in lines[i + 13]:
            # Znajdź indeks końca frazy 'baseColor'
            index = line.find("baseColor") + len("baseColor")
            # Usuń wszystko po 'baseColor' do końca linii
            line = line[:index]
            if kolory:  # Sprawdź, czy w puli są jeszcze jakieś kolory
                # Wybierz losowy kolor z puli
                wybrany_kolor = random.choice(kolory)
                # Usuń wybrany kolor z puli
                kolory.remove(wybrany_kolor)
                # Dodaj wybrany kolor do linii
                line += wybrany_kolor
            else:
                print("Nie ma więcej kolorów w puli.")
    
    # Dodaj zmodyfikowaną lub oryginalną linię do listy
    zmodyfikowane_linie.append(line)

# Otwórz ten sam plik w trybie zapisu, aby zapisać zmodyfikowaną zawartość
with open('mars_mines.wbt', 'w') as file:
    file.writelines(zmodyfikowane_linie)

print("Zakończono modyfikację i zapisano plik.")
