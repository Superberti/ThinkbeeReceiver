## ThinkbeeReceiver - Thinkbee Funkprotokoll Empfänger

Dieses kleine Demonstrationsprogramm kann das Funkprotokoll der s.g. Thinkbee-Schalter demodulieren.
Als Hardware wurde ein Nucleo-32 (STM32L031) verwendet, da gerade vorhanden. 
Als Empfänger dient ein billiger RXB6-433-MHz-Empfänger, Data-Out ist an PA1 des Nucleos angeschlossen.
Als Sender dient ein Thinkbee Ein/Aus-Doppelschalter.

Bei dem Demoprogramm handelt sich um ein EmBitz 2.0-Projekt mit der HAL V1.12.2.

Dieser Empfänger hat keine Rauschsperre und sendet ständig Daten. Beim Empfang des Thinkbee-Signals wird dann
die Verstärkung automatisch herabgesetzt, damit das Signal nicht übersteuert. Das dauert aber etwas und dabei
geht die Start-Sequenz schon mal verloren. Glücklicherweise wird die Sequenz drei mal wiederholt, sodass man sich
auch noch später darauf synchronisieren kann.

Zur Bestimmung der Bitlaufzeiten wird der Capture-Timer (unterschiedliche Interrrupts für jede Flanke) des STMs benutzt und dann schrittweise auf Gültigkeit geprüft.
Heraus kommt am Ende ein Datenwort (34 Bits), mit dem man zumindest schon einmal die Schalter Ein/Aus unterscheiden kann.
Welche Bits zu einer ID oder zu einer Funktion gehören, kann ich mit meinen vorhandenen zwei Schaltern natürlich nicht
feststellen, hier wäre ich für weiteren Input (=aufgezeichnete Signale) dankbar!

Die Datenerfassung ist zwar momentan recht STM32-spezifisch, die DecodeStart- und DecodeMsg-Funktionen benötigen allerdings nur
den aktuellen Eingangslevel (nach Flankenwechsel), die Zeit bis zum vorherigen Flankenwechsel und einen Schrittzähler. Damit sollte man
auch mit anderen Mikrocontrollern kein Problem haben.

Viel Spaß beim Ausprobieren!

 
