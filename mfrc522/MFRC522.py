import time
from machine import Pin, SPI


class AuthenticationError(Exception):
    pass


class StatusNotSuccessError(Exception):
    pass


class MFRC522:
    MAX_LEN = 16

    PCD_IDLE = 0x00
    PCD_AUTHENT = 0x0E
    PCD_RECEIVE = 0x08
    PCD_TRANSMIT = 0x04
    PCD_TRANSCEIVE = 0x0C
    PCD_RESETPHASE = 0x0F
    PCD_CALCCRC = 0x03

    PICC_REQIDL = 0x26
    PICC_REQALL = 0x52
    PICC_ANTICOLL = 0x93
    PICC_SElECTTAG = 0x93
    PICC_AUTHENT1A = 0x60
    PICC_AUTHENT1B = 0x61
    PICC_READ = 0x30
    PICC_WRITE = 0xA0
    PICC_DECREMENT = 0xC0
    PICC_INCREMENT = 0xC1
    PICC_RESTORE = 0xC2
    PICC_TRANSFER = 0xB0
    PICC_HALT = 0x50

    MI_OK = 0
    MI_NOTAGERR = 1
    MI_ERR = 2

    Reserved00 = 0x00
    CommandReg = 0x01
    CommIEnReg = 0x02
    DivlEnReg = 0x03
    CommIrqReg = 0x04
    DivIrqReg = 0x05
    ErrorReg = 0x06
    Status1Reg = 0x07
    Status2Reg = 0x08
    FIFODataReg = 0x09
    FIFOLevelReg = 0x0A
    WaterLevelReg = 0x0B
    ControlReg = 0x0C
    BitFramingReg = 0x0D
    CollReg = 0x0E
    Reserved01 = 0x0F

    Reserved10 = 0x10
    ModeReg = 0x11
    TxModeReg = 0x12
    RxModeReg = 0x13
    TxControlReg = 0x14
    TxAutoReg = 0x15
    TxSelReg = 0x16
    RxSelReg = 0x17
    RxThresholdReg = 0x18
    DemodReg = 0x19
    Reserved11 = 0x1A
    Reserved12 = 0x1B
    MifareReg = 0x1C
    Reserved13 = 0x1D
    Reserved14 = 0x1E
    SerialSpeedReg = 0x1F

    Reserved20 = 0x20
    CRCResultRegM = 0x21
    CRCResultRegL = 0x22
    Reserved21 = 0x23
    ModWidthReg = 0x24
    Reserved22 = 0x25
    RFCfgReg = 0x26
    GsNReg = 0x27
    CWGsPReg = 0x28
    ModGsPReg = 0x29
    TModeReg = 0x2A
    TPrescalerReg = 0x2B
    TReloadRegH = 0x2C
    TReloadRegL = 0x2D
    TCounterValueRegH = 0x2E
    TCounterValueRegL = 0x2F

    Reserved30 = 0x30
    TestSel1Reg = 0x31
    TestSel2Reg = 0x32
    TestPinEnReg = 0x33
    TestPinValueReg = 0x34
    TestBusReg = 0x35
    AutoTestReg = 0x36
    VersionReg = 0x37
    AnalogTestReg = 0x38
    TestDAC1Reg = 0x39
    TestDAC2Reg = 0x3A
    TestADCReg = 0x3B
    Reserved31 = 0x3C
    Reserved32 = 0x3D
    Reserved33 = 0x3E
    Reserved34 = 0x3F

    serNum = []

    def __init__(self, spi_bus, select_pin: int, reset_pin: int) -> None:
        """
        Initialise une instance du lecteur MFRC522 via SPI.
        
        :param spi_bus: Instance du bus SPI utilisé pour communiquer avec le MFRC522.
        :param select_pin: Numéro de la pin utilisée pour la sélection (chip select).
        :param reset_pin: Numéro de la pin de réinitialisation du MFRC522.
        """
        self.spi_bus = spi_bus
        self.spi_bus.init()  # Initialisation du bus SPI

        # Configuration des pins de réinitialisation et de sélection comme sorties
        self.reset_pin = Pin(reset_pin, Pin.OUT)
        self.select_pin = Pin(select_pin, Pin.OUT)
        
        # Met le pin de réinitialisation à l'état haut pour activer le module
        self.reset_pin.value(True)
        
        # Initialise le module MFRC522 (configuration des registres, activation de l'antenne, etc.)
        self.MFRC522_Init()
            

    def MFRC522_Reset(self) -> None:
        """
        Réinitialise le module MFRC522 en envoyant la commande de réinitialisation (PCD_RESETPHASE)
        au registre de commande (CommandReg).
        """
        self.Write_MFRC522(self.CommandReg, self.PCD_RESETPHASE)
        
        
    def Write_MFRC522(self, addr: int, val: int) -> None:
        """
        Écrit une valeur dans le registre spécifié du MFRC522 via SPI.
        
        Le protocole SPI du MFRC522 nécessite que l'octet d'adresse soit formaté ainsi :
          - Bit 7 (MSB) : 0 pour écriture.
          - Bits 6 à 1 : adresse du registre.
          - Bit 0 (LSB) : réservé, doit être 0.
        
        Pour cela, on décale l'adresse vers la gauche d'une position, on masque avec 0x7e pour s'assurer
        que le MSB et le LSB sont à 0, puis on envoie cet octet suivi de la valeur.
        
        :param addr: Adresse du registre (0-127).
        :param val: Valeur à écrire dans le registre.
        """
        # Active la sélection du périphérique
        self.select_pin.value(0)
        
        # Formate l'octet d'adresse pour une écriture : décalage à gauche et masque pour MSB=0, LSB=0.
        address_byte = (addr << 1) & 0x7e
        self.spi_bus.write(bytes([address_byte]))
        
        # Envoie la valeur à écrire
        self.spi_bus.write(bytes([val & 0xff]))
        
        # Désactive la sélection
        self.select_pin.value(1)

    def Read_MFRC522(self, addr: int) -> int:
        """
        Lit la valeur d'un registre du MFRC522 via SPI.
        
        Le protocole SPI du MFRC522 nécessite que l'octet d'adresse soit formaté ainsi :
          - Bit 7 (MSB) : 1 pour lecture.
          - Bits 6 à 1 : adresse du registre.
          - Bit 0 (LSB) : réservé, doit être 0.
        
        On décalera l'adresse à gauche d'une position, puis on active le bit de lecture (OR 0x80).
        Après avoir envoyé cet octet, on lit 1 octet, qui correspond à la valeur du registre demandé.
        
        :param addr: Adresse du registre à lire (0-127).
        :return: Valeur lue depuis le registre (0-255).
        """
        # Active la sélection du périphérique
        self.select_pin.value(0)
        
        # Formate l'octet d'adresse pour une lecture : décalage à gauche et mise du MSB à 1.
        address_byte = ((addr << 1) & 0x7e) | 0x80
        self.spi_bus.write(bytes([address_byte]))
        
        # Lit un octet depuis le bus SPI (la première réponse utile)
        result = self.spi_bus.read(1)
        
        # Désactive la sélection du périphérique
        self.select_pin.value(1)
        
        return result[0]

    def Close_MFRC522(self) -> None:
        """
        Ferme la connexion SPI avec le module MFRC522.
        """
        self.spi.close()

    def SetBitMask(self, reg: int, mask: int) -> None:
        """
        Active (met à 1) les bits spécifiés par 'mask' dans le registre 'reg'.
        
        :param reg: Adresse du registre.
        :param mask: Masque des bits à activer.
        """
        current_value = self.Read_MFRC522(reg)
        self.Write_MFRC522(reg, current_value | mask)

    def ClearBitMask(self, reg: int, mask: int) -> None:
        """
        Désactive (met à 0) les bits spécifiés par 'mask' dans le registre 'reg'.
        
        :param reg: Adresse du registre.
        :param mask: Masque des bits à désactiver.
        """
        current_value = self.Read_MFRC522(reg)
        self.Write_MFRC522(reg, current_value & (~mask))

    def AntennaOn(self) -> None:
        """
        Active l'antenne du module MFRC522.
        
        Vérifie si les bits de contrôle de l'antenne (bits 0 et 1 dans TxControlReg)
        ne sont pas déjà activés, puis les active si nécessaire.
        """
        tx_control = self.Read_MFRC522(self.TxControlReg)
        # Si aucun des bits 0x03 n'est activé, on active l'antenne.
        if not (tx_control & 0x03):
            self.SetBitMask(self.TxControlReg, 0x03)

    def AntennaOff(self) -> None:
        """
        Désactive l'antenne du module MFRC522 en effaçant les bits 0x03 du registre TxControlReg.
        """
        self.ClearBitMask(self.TxControlReg, 0x03)

    def MFRC522_ToCard(self, command: int, sendData: list) -> tuple:
        """
        Envoie une commande au module MFRC522 avec les données fournies, puis attend et lit la réponse.
        
        :param command: La commande à exécuter (par exemple, self.PCD_AUTHENT ou self.PCD_TRANSCEIVE).
        :param sendData: La liste des octets à envoyer (ex. [0x93, 0x20] pour une commande d'anticollision).
        :return: Un tuple (status, backData, backLen) où :
                 - status est self.MI_OK, self.MI_NOTAGERR ou self.MI_ERR,
                 - backData est la liste des octets reçus,
                 - backLen est la longueur de la réponse en bits.
        """
        # Initialisation des variables de résultat
        backData = []
        backLen = 0
        status = self.MI_ERR

        # Définition de constantes locales pour clarifier le code
        GLOBAL_IRQ_MASK = 0x80          # Bit pour activer globalement les IRQ
        TIMEOUT_LIMIT = 2000            # Nombre maximal d'itérations pour le timeout
        ERROR_MASK = 0x1B               # Masque pour vérifier les erreurs dans ErrorReg
        START_TRANSCEIVE_MASK = 0x80    # Bit à activer pour lancer la transmission dans BitFramingReg

        # Configuration des masques d'IRQ en fonction de la commande
        if command == self.PCD_AUTHENT:
            irqEn = 0x12
            waitIRq = 0x10
        elif command == self.PCD_TRANSCEIVE:
            irqEn = 0x77
            waitIRq = 0x30
        else:
            raise ValueError("Commande non supportée")

        # Activation des IRQs et préparation du module
        self.Write_MFRC522(self.CommIEnReg, irqEn | GLOBAL_IRQ_MASK)
        self.ClearBitMask(self.CommIrqReg, GLOBAL_IRQ_MASK)
        self.SetBitMask(self.FIFOLevelReg, GLOBAL_IRQ_MASK)  # Réinitialise (flush) le FIFO

        # Mise en mode IDLE pour annuler toute commande précédente
        self.Write_MFRC522(self.CommandReg, self.PCD_IDLE)

        # Chargement des données dans le FIFO
        for dataByte in sendData:
            self.Write_MFRC522(self.FIFODataReg, dataByte)

        # Lancement de la commande
        self.Write_MFRC522(self.CommandReg, command)
        if command == self.PCD_TRANSCEIVE:
            self.SetBitMask(self.BitFramingReg, START_TRANSCEIVE_MASK)

        # Boucle d'attente basée sur les IRQ
        timeout = TIMEOUT_LIMIT
        irqValue = 0
        while timeout > 0:
            irqValue = self.Read_MFRC522(self.CommIrqReg)
            # On sort de la boucle si le bit de timer (0x01) ou l'une des IRQ attendues est levée
            if (irqValue & 0x01) or (irqValue & waitIRq):
                break
            timeout -= 1

        # Réinitialise le bit de démarrage de transmission
        self.ClearBitMask(self.BitFramingReg, START_TRANSCEIVE_MASK)

        # Si le timeout est atteint, on retourne une erreur
        if timeout == 0:
            status = self.MI_ERR
            return (status, backData, backLen)

        # Vérification des erreurs dans ErrorReg
        errorRegValue = self.Read_MFRC522(self.ErrorReg)
        if (errorRegValue & ERROR_MASK) == 0:
            status = self.MI_OK
            # Si une interruption particulière est détectée, on considère qu'aucun tag n'est présent
            if irqValue & irqEn & 0x01:
                status = self.MI_NOTAGERR

            # Pour une commande de type TRANSCEIVE, lecture des données reçues depuis le FIFO
            if command == self.PCD_TRANSCEIVE:
                nBytes = self.Read_MFRC522(self.FIFOLevelReg)
                lastBits = self.Read_MFRC522(self.ControlReg) & 0x07

                # Calcul du nombre total de bits reçus
                if lastBits != 0:
                    backLen = (nBytes - 1) * 8 + lastBits
                else:
                    backLen = nBytes * 8

                # Correction du nombre d'octets si nécessaire
                if nBytes == 0:
                    nBytes = 1
                if nBytes > self.MAX_LEN:
                    nBytes = self.MAX_LEN

                for _ in range(nBytes):
                    backData.append(self.Read_MFRC522(self.FIFODataReg))
        else:
            status = self.MI_ERR

        return (status, backData, backLen)

    def MFRC522_Request(self, reqMode: int) -> tuple:
        """
        Envoie une commande de requête (REQA ou WUPA) au module MFRC522 pour détecter une carte RFID.
        
        :param reqMode: Le mode de requête à envoyer (par exemple, 0x26 pour REQA).
        :return: Un tuple (status, backBits) où status est MI_OK ou MI_ERR,
                 et backBits représente le nombre de bits reçus (devrait être 16 en cas de succès).
        """
        # Configure le BitFramingReg pour transmettre 7 bits (format requis pour REQA/WUPA)
        self.Write_MFRC522(self.BitFramingReg, 0x07)

        # Prépare le type de tag en ajoutant le mode de requête dans la liste
        tagType = [reqMode]

        # Envoie la commande en mode TRANSCEIVE et récupère la réponse du tag
        status, backData, backBits = self.MFRC522_ToCard(self.PCD_TRANSCEIVE, tagType)

        # Vérifie si le status est OK et si le nombre de bits reçus est exactement 16 (0x10)
        if status != self.MI_OK or backBits != 0x10:
            status = self.MI_ERR

        return (status, backBits)

    def MFRC522_Anticoll(self) -> tuple:
        """
        Lance la procédure d'anticollision pour récupérer l'UID d'une carte RFID.
        
        La fonction envoie la commande d'anticollision au MFRC522 et attend une réponse de 5 octets,
        où les 4 premiers octets correspondent à l'UID et le 5ème est le BCC (Block Check Character).
        Le BCC est vérifié en faisant un XOR des 4 premiers octets.
        
        :return: Un tuple (status, backData) où :
                 - status est self.MI_OK si l'opération réussit, sinon self.MI_ERR.
                 - backData est la liste des octets reçus.
        """
        # Initialisation des variables
        backData = []
        calculatedBCC = 0

        # Prépare la commande anticollision
        uidCommand = []
        
        # Réinitialise le BitFramingReg pour que la trame soit complète (aucun décalage de bits)
        self.Write_MFRC522(self.BitFramingReg, 0x00)
        
        # La commande anticollision se compose du code PICC_ANTICOLL et du paramètre 0x20
        uidCommand.append(self.PICC_ANTICOLL)  # Par exemple, souvent 0x93
        uidCommand.append(0x20)

        # Envoie la commande en mode TRANSCEIVE et récupère la réponse
        status, backData, backBits = self.MFRC522_ToCard(self.PCD_TRANSCEIVE, uidCommand)

        # Si la commande a réussi, vérifier la validité des données reçues
        if status == self.MI_OK:
            # La réponse attendue doit être de 5 octets : 4 octets UID + 1 octet BCC
            if len(backData) == 5:
                # Calcul du BCC en faisant le XOR des 4 premiers octets
                for i in range(4):
                    calculatedBCC ^= backData[i]
                # Vérifie que le BCC calculé correspond au 5ème octet reçu
                if calculatedBCC != backData[4]:
                    status = self.MI_ERR
            else:
                status = self.MI_ERR

        return (status, backData)

    def CalculateCRC(self, inputData: list) -> list:
        """
        Calcule le CRC sur les données fournies en utilisant le calcul CRC interne du MFRC522.
        
        La fonction prépare le module en effaçant le bit CRC de DivIrqReg et en vidant le FIFO.
        Elle écrit ensuite les données dans le FIFO, lance la commande PCD_CALCCRC et attend la fin du calcul
        (signalée par le bit CRC dans DivIrqReg). Le résultat est lu dans CRCResultRegL et CRCResultRegM.
        
        :param inputData: Liste d'octets sur lesquels le CRC doit être calculé.
        :return: Une liste contenant deux octets, [CRC_L, CRC_M].
        """
        # Constantes pour clarifier le code
        CRC_IRQ_BIT = 0x04        # Bit dans DivIrqReg indiquant la fin du calcul CRC
        FIFO_FLUSH_MASK = 0x80      # Bit pour vider le FIFO
        TIMEOUT_LIMIT = 0xFF        # Nombre maximal d'itérations en attente

        # Prépare le module : efface le bit CRC et vide le FIFO
        self.ClearBitMask(self.DivIrqReg, CRC_IRQ_BIT)
        self.SetBitMask(self.FIFOLevelReg, FIFO_FLUSH_MASK)

        # Charge les données dans le FIFO
        for byte in inputData:
            self.Write_MFRC522(self.FIFODataReg, byte)

        # Lance le calcul CRC
        self.Write_MFRC522(self.CommandReg, self.PCD_CALCCRC)

        # Attente que le bit CRC soit levé dans DivIrqReg ou que le timeout soit atteint
        timeout = TIMEOUT_LIMIT
        while timeout > 0:
            irq_value = self.Read_MFRC522(self.DivIrqReg)
            if irq_value & CRC_IRQ_BIT:
                break
            timeout -= 1

        # Lecture du résultat CRC sur 2 octets
        crcResult = [
            self.Read_MFRC522(self.CRCResultRegL),
            self.Read_MFRC522(self.CRCResultRegM)
        ]
        return crcResult

    def MFRC522_SelectTag(self, serialNumber: list) -> int:
        """
        Sélectionne un tag (carte RFID) à partir de son UID obtenu via l'anticollision.
        
        La fonction construit une trame de sélection composée de :
          - La commande SELECT (PICC_SElECTTAG)
          - Le paramètre NVB (0x70)
          - Les 5 octets de l'UID (incluant le BCC obtenu lors de l'anticollision)
          - Les 2 octets CRC calculés sur l'ensemble des données précédentes.
        
        La commande est envoyée en mode TRANSCEIVE, et la réponse attendue doit comporter 24 bits (0x18).
        En cas de succès, le premier octet de la réponse (souvent le SAK ou la taille du tag) est renvoyé.
        
        :param serialNumber: Liste de 5 octets correspondant à l'UID + BCC d'un tag.
        :return: Le premier octet de la réponse (typiquement la taille ou le SAK) si la sélection est réussie,
                 sinon 0.
        """
        backData = []
        commandBuffer = []
        
        # Construction de la trame de sélection
        commandBuffer.append(self.PICC_SElECTTAG)  # Commande SELECT
        commandBuffer.append(0x70)                # NVB : indique le nombre d'octets valides à suivre (0x70 standard)
        
        # Ajout de l'UID (5 octets : UID + BCC)
        for i in range(5):
            commandBuffer.append(serialNumber[i])
        
        # Calcul du CRC sur la trame construite et ajout des deux octets CRC
        crcResult = self.CalculateCRC(commandBuffer)
        commandBuffer.append(crcResult[0])
        commandBuffer.append(crcResult[1])
        
        # Envoi de la commande en mode TRANSCEIVE
        status, backData, backLen = self.MFRC522_ToCard(self.PCD_TRANSCEIVE, commandBuffer)
        
        # Vérification de la réponse : on attend 24 bits (0x18) pour une commande SELECT réussie
        if (status == self.MI_OK) and (backLen == 0x18):
            print("Size: " + str(backData[0]))
            return backData[0]
        else:
            return 0

    def MFRC522_StopCrypto1(self) -> None:
        """
        Désactive le chiffrement (Crypto1) sur la communication avec la carte RFID.
        
        Le bit 0x08 du registre Status2Reg indique que Crypto1 est actif.
        En effaçant ce bit, on arrête le chiffrement.
        """
        self.ClearBitMask(self.Status2Reg, 0x08)

    def MFRC522_Auth(self, authMode: int, blockAddr: int, sectorKey: list, serNum: list) -> int:
        """
        Authentifie l'accès à un bloc spécifique d'une carte RFID.
        
        La fonction construit un buffer d'authentification composé de :
          - authMode : mode d'authentification (clé A ou B),
          - blockAddr : l'adresse du bloc à authentifier,
          - sectorKey : la clé d'authentification (généralement 6 octets),
          - serNum : les 4 premiers octets de l'UID de la carte.
          
        Ce buffer est envoyé via la commande PCD_AUTHENT.
        La réussite de l'authentification est vérifiée en s'assurant que :
          - Le statut retourné est MI_OK,
          - Le bit 0x08 dans le registre Status2Reg est levé, indiquant que le chiffrement (Crypto1) est actif.
          
        :param authMode: Mode d'authentification (par exemple, clé A ou clé B).
        :param blockAddr: Adresse du bloc à authentifier.
        :param sectorKey: Liste de 6 octets représentant la clé d'authentification.
        :param serNum: Liste d'octets représentant l'UID de la carte (les 4 premiers octets seront utilisés).
        :return: Le statut de l'authentification (MI_OK en cas de succès).
        :raises AuthenticationError: En cas d'échec de l'authentification.
        """
        # Construction du buffer d'authentification
        buffer = [authMode, blockAddr]
        buffer.extend(sectorKey)
        buffer.extend(serNum[:4])
        
        # Envoi de la commande d'authentification
        status, backData, backLen = self.MFRC522_ToCard(self.PCD_AUTHENT, buffer)
        
        # Vérification du statut de la commande
        if status != self.MI_OK:
            raise AuthenticationError(f"AUTH ERROR: Command failed with status {status}")
        
        # Vérifie que le bit 0x08 dans Status2Reg est activé (indiquant le succès de l'authentification)
        if (self.Read_MFRC522(self.Status2Reg) & 0x08) == 0:
            raise AuthenticationError("AUTH ERROR: (Status2Reg & 0x08) not set")
        
        return status

    def MFRC522_Read(self, blockAddr: int) -> list:
        """
        Lit un bloc de données sur une carte RFID à partir de l'adresse spécifiée.
        
        La fonction construit une trame de lecture en ajoutant la commande PICC_READ et l'adresse
        du bloc à lire, calcule le CRC pour garantir l'intégrité de la trame, puis envoie la commande
        en mode TRANSCEIVE. La réponse attendue est un bloc de 16 octets.
        
        :param blockAddr: Adresse du bloc à lire sur la carte RFID.
        :return: Une liste contenant 16 octets si la lecture est réussie, sinon None.
        :raises StatusNotSuccessError: Si le statut retourné par MFRC522_ToCard n'est pas MI_OK.
        """
        # Construction de la trame de lecture avec la commande et l'adresse du bloc
        commandBuffer = [self.PICC_READ, blockAddr]
        
        # Calcul du CRC sur la trame et ajout des deux octets de CRC à la fin
        crcResult = self.CalculateCRC(commandBuffer)
        commandBuffer.extend(crcResult)
        
        # Envoi de la commande en mode TRANSCEIVE et récupération de la réponse
        status, backData, backLen = self.MFRC522_ToCard(self.PCD_TRANSCEIVE, commandBuffer)
        
        # Vérifie que la commande s'est bien déroulée
        if status != self.MI_OK:
            raise StatusNotSuccessError("Error while reading!")
        
        # Vérifie que la réponse contient exactement 16 octets (taille d'un bloc de données)
        if len(backData) == 16:
            print("Sector " + str(blockAddr) + " " + str(backData))
            return backData
        else:
            return None

    def MFRC522_Write(self, blockAddr: int, writeData: list) -> None:
        """
        Écrit un bloc de 16 octets sur une carte RFID dans le bloc spécifié par blockAddr.
        
        La procédure se déroule en deux étapes :
          1. Envoi de la commande d'écriture (WRITE) avec l'adresse du bloc et vérification de l'accusé de réception.
          2. Si le tag est prêt, envoi du bloc de 16 octets à écrire (writeData) accompagné du CRC calculé.
        
        :param blockAddr: L'adresse du bloc sur lequel écrire.
        :param writeData: Une liste de 16 octets à écrire.
        :raises StatusNotSuccessError: En cas d'erreur pendant la commande d'écriture ou si le tag ne confirme pas.
        :raises ValueError: Si writeData ne contient pas au moins 16 octets.
        """
        # --- Étape 1 : Envoi de la commande d'écriture ---
        # Construction du buffer de commande : [PICC_WRITE, blockAddr, CRC1, CRC2]
        commandBuffer = [self.PICC_WRITE, blockAddr]
        crc = self.CalculateCRC(commandBuffer)
        commandBuffer.extend(crc)
        
        status, backData, backLen = self.MFRC522_ToCard(self.PCD_TRANSCEIVE, commandBuffer)
        
        # Vérification de l'accusé de réception pour la commande WRITE
        # On attend un retour de 4 bits et que le premier octet (masqué) soit égal à 0x0A.
        if status != self.MI_OK or backLen != 4 or (backData[0] & 0x0F) != 0x0A:
            status = self.MI_ERR

        print(f"{backLen} backdata &0x0F == 0x0A {(backData[0] & 0x0F)}")
        
        # --- Étape 2 : Transmission des données à écrire ---
        if status == self.MI_OK:
            # Vérifie que writeData contient 16 octets
            if len(writeData) < 16:
                raise ValueError("writeData doit contenir au moins 16 octets")
            
            # Préparation du buffer de données (16 octets) à écrire
            dataBuffer = writeData[:16]  # On prend les 16 premiers octets
            crc = self.CalculateCRC(dataBuffer)
            dataBuffer.extend(crc)
            
            status, backData, backLen = self.MFRC522_ToCard(self.PCD_TRANSCEIVE, dataBuffer)
            
            # Vérification de l'accusé de réception pour l'écriture des données
            if status != self.MI_OK or backLen != 4 or (backData[0] & 0x0F) != 0x0A:
                raise StatusNotSuccessError("Error while writing")
            
            print("Data written")


    def MFRC522_DumpClassic1K(self, key: list, uid: list) -> dict:
        """
        Lit et renvoie les données de tous les 64 blocs d'une carte MIFARE Classic 1K.
        
        Pour chaque bloc (de 0 à 63), la fonction tente d'authentifier le bloc en utilisant
        la clé (key) et l'UID (uid). En cas d'authentification réussie, le contenu du bloc est lu
        via MFRC522_Read et stocké dans un dictionnaire dont la clé est l'adresse du bloc.
        
        :param key: Liste de 6 octets représentant la clé d'authentification.
        :param uid: Liste d'octets représentant l'UID du tag.
        :return: Un dictionnaire {block_address: block_data, ...} avec les données de chaque bloc.
        :raises AuthenticationError: Si l'authentification échoue pour un bloc.
        """
        dump_data = {}
        
        # Parcours de tous les blocs (0 à 63)
        for blockAddr in range(64):
            status = self.MFRC522_Auth(self.PICC_AUTHENT1A, blockAddr, key, uid)
            
            # Si l'authentification est réussie, on lit le bloc et on stocke le résultat
            if status == self.MI_OK:
                block_data = self.MFRC522_Read(blockAddr)
                dump_data[blockAddr] = block_data
            else:
                # En cas d'échec d'authentification, on lève une exception avec l'adresse du bloc problématique
                raise AuthenticationError(f"Authentication error on block {blockAddr}")
        
        return dump_data

    def MFRC522_Init(self) -> None:
        """
        Initialise le module MFRC522 en réinitialisant le module et en configurant ses registres internes.
        
        Les étapes de configuration comprennent :
          - La réinitialisation du module via MFRC522_Reset.
          - La configuration du timer interne (TModeReg, TPrescalerReg, TReloadRegL, TReloadRegH)
            pour gérer les délais et le timeout.
          - La configuration du registre TxAutoReg pour l'émission automatique.
          - La configuration du registre ModeReg pour définir le mode de fonctionnement du MFRC522.
          - L'activation de l'antenne avec AntennaOn, essentielle pour la communication RFID.
        
        Ces paramètres sont recommandés par le datasheet pour un fonctionnement optimal du MFRC522.
        """
        # Réinitialisation du module
        self.MFRC522_Reset()

        # Configuration du timer interne
        self.Write_MFRC522(self.TModeReg, 0x8D)
        self.Write_MFRC522(self.TPrescalerReg, 0x3E)
        self.Write_MFRC522(self.TReloadRegL, 30)
        self.Write_MFRC522(self.TReloadRegH, 0)

        # Configuration du mode de transmission et de fonctionnement
        self.Write_MFRC522(self.TxAutoReg, 0x40)
        self.Write_MFRC522(self.ModeReg, 0x3D)

        # Activation de l'antenne
        self.AntennaOn()

