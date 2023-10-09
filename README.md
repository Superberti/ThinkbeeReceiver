## ThinkbeeReceiver
#Thinkbee Funkprotokoll Empfänger

Dieses kleine Demonstrationsprogramm kann das Funkprotokoll der s.g. Thinkbee-Schalter demodulieren.
Als Hardware wurde ein Nucleo-32 (STM32L031) verwendet, da gerade vorhanden. 
Als Empfänger dient ein billiger RXB6-433-MHz-Empfänger, Data-Out ist an PA1 des Nucleos angeschlossen.
Als Sender dient ein Thinkbee Ein/Aus-Doppelschalter.

Dieser Empfänger hat keine Rauschsperre und sendet ständig Daten. Beim Empfang des Thinkbee-Signals wird dann
die Verstärkung automatisch herabgesetzt, damit das Signal nicht übersteuert. Das dauert aber etwas und dabei
geht die Start-Sequenz schon mal verloren. Glücklicherweise wird die Sequenz drei mal wiederholt, sodass man sich
auch noch später darauf synchronisieren kann.
Zur Bestimmung der Bitlaufzeiten wird der Capture-Timer des STMs benutzt und dann schrittweise auf Gültigkeit geprüft.
Heraus kommt am Ende ein Datenwort (34 Bits), mit dem man zumindest schon einmal die Schalter Ein/Aus unterscheiden kann.
Welche Bits zu einer ID oder zu einer Funktion gehören, kann ich mit meinen vorhandenen ztwei Schaltern natürlich nicht
feststellen, hier wäre ich für weiteren Input (=aufgezeichnete Signale) dankbar!

Viel Spaß beim Ausprobieren!

 
