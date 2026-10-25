# CyberDog: app di controllo v2

Una pagina nel browser del PC per guidare il CyberDog 2, fargli fare dei movimenti, regolare luci e postura, vedere la fotocamera e usare le mappe. Il programma gira solo sul tuo PC (indirizzo `127.0.0.1`), e nessun altro computer della rete può usarlo.

## Avvio

1. Accendi il cane e collega il PC al cane come al solito (hotspot del PC, indirizzo del cane `192.168.137.115`).
2. Apri PowerShell nella cartella `CyberDog` e scrivi:

   ```powershell
   .\app2\run.ps1
   ```

3. Il browser si apre da solo su **http://127.0.0.1:8794/**. Se non si apre, copia l'indirizzo dalla finestra di PowerShell.
4. Per chiudere tutto, premi **Ctrl+C** nella finestra di PowerShell.

La pagina mostra se il cane è collegato, la batteria e il tempo di risposta.

## L'interruttore del movimento

- All'avvio l'interruttore è **spento**. Si spegne anche ogni volta che ricarichi la pagina.
- Finché è spento, la pagina rifiuta ogni nuovo comando di guida, movimento, postura e navigazione.
  Luci, volume, sensori e fotocamera funzionano lo stesso.
- Quando lo spegni, la camminata e la postura si fermano subito, e i movimenti in attesa vengono annullati.
  Un movimento già partito sul cane va fino in fondo, e così un viaggio verso un punto o verso la base di ricarica:
  per fermarli subito usa l'arresto di emergenza, oppure «Ferma navigazione» per il viaggio.

## Arresto di emergenza

Il pulsante **ARRESTO DI EMERGENZA**, o il tasto **Spazio**, funziona sempre, anche con l'interruttore spento o con un altro movimento in corso.

- Il cane interrompe subito quello che sta facendo e può abbassarsi di colpo.
- Dopo l'arresto, l'interruttore torna spento.
- Per ripartire, riaccendi l'interruttore e premi «alzati» (tasto **1**).

Anche lasciare un tasto di guida ferma il cane: se per 0,4 secondi non arriva nessun comando (tasto rilasciato, pagina chiusa, Wi-Fi caduto), il cane si ferma da solo.

## Tasti

| Tasto | Azione |
|---|---|
| W / S | avanti / indietro (tieni premuto) |
| A / D | gira a sinistra / a destra |
| Q / E | passo laterale a sinistra / a destra |
| Spazio | arresto di emergenza |
| G | cambia andatura (lenta, media, veloce, automatica) |
| + / - | più veloce / più lento |
| 1 | alzati / siediti |
| 2-9 | movimenti: 141, 123, 146, 144, 145, 174, 173, 140 (raccolta di balli) |

All'avvio la velocità è bassa: con l'andatura lenta il cane va al massimo a 0,2 m/s.

## «Pavimento libero?»

Alcuni movimenti si spostano molto o durano a lungo, per esempio la raccolta di balli (tasto **9**, circa 60 secondi). Prima di eseguirli la pagina chiede **«pavimento libero?»**. Conferma solo se intorno al cane c'è almeno 1,5 m di spazio libero, senza persone, animali, scale o oggetti fragili.

I movimenti pericolosi, come salti e capriole, non sono disponibili in questa app.

La **passeggiata al guinzaglio** (125, nella raccolta dei movimenti) resta attiva finché non dai un altro comando di movimento o premi l'arresto di emergenza: il cane non torna da solo alla posizione normale.

## Una scheda alla volta

Apri la pagina in una sola scheda del browser. Se la apri in una seconda scheda, quella nuova mostra «L'app è aperta in un'altra scheda»: da lì puoi sempre premere **STOP** (o Spazio), oppure **Usa questa scheda** per passare il comando a quella scheda. Quando passi di scheda, l'interruttore del movimento si spegne. Un viaggio verso un punto o verso la base già partito continua: la nuova scheda lo mostra e lo può fermare.

Il motivo: il browser apre al massimo 6 collegamenti verso l'app, e due schede con fotocamera e aggiornamenti accesi possono occuparli tutti, così l'arresto di emergenza resterebbe in coda.

## Provare senza il cane (emulatore)

L'emulatore è un cane finto che gira sul PC. È utile per provare la pagina senza rischi.

1. In una prima finestra di PowerShell, nella cartella `CyberDog`, avvia l'emulatore:

   ```powershell
   $env:PYTHONPATH = "$PWD\pylibs;$PWD"
   python -m emulator --port 50052
   ```

2. In una seconda finestra, avvia l'app collegandola all'emulatore:

   ```powershell
   .\app2\run.ps1 -Dog 127.0.0.1:50052
   ```

La pagina indica che il collegamento è con l'emulatore e non con il cane vero. Per tornare al cane vero, chiudi l'app con Ctrl+C e avviala di nuovo senza `-Dog`.

## Se qualcosa non va

- **«il cane non risponde»**: controlla che il cane sia acceso e collegato all'hotspot. L'app si ricollega da sola.
- **«movimento disabilitato»**: accendi l'interruttore del movimento.
- **«un movimento è già in corso»**: aspetta che finisca, oppure usa l'arresto di emergenza.
- **La porta 8794 è già in uso**: c'è già un'app aperta. Chiudila, oppure avvia questa su un'altra porta con `.\app2\run.ps1 -Port 8795`.
- **La mappa non compare mentre la crei**: è normale. Il cane manda la mappa solo dopo «Salva mappa»; fino ad allora la pagina mostra l'ultima mappa salvata.
