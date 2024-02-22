import argparse
import sys
import os
import hashlib
import ctypes
import base64

hexData=None
lastLineNumber=0
checksumLineNumber=0
sidLineNumber=0
fw_1_md_row_index=0
fw_2_md_row_index=0
hex_file_row_size = 0x40
#Row size of PMG1S3
device_row_size = 0x100
#Flash size of PMG1S3
device_flash_size = 0x40000
#Size of application siganture
cy_signature_size = 0x20

# Part list information file (including path) The file is
# expected to reside in the same location as the script.
partListFile="partList.info"
# This has to be updated for every script to last supported partList information
partListMinVersionSupport={'major_version': 1,'minor_version':1, 'patch_version':2}
partListMaxVersionSupport={'major_version': 1,'minor_version':1, 'patch_version':2}

part_list_version=None
part_list=None
unknown_part=None
part_info=None

#Based on existing Scripts
def parseInputFile (input_file):

    global hexData
    global sidLineNumber, checksumLineNumber, lastLineNumber, fw_1_md_row_index, fw_2_md_row_index

    f = open (input_file, 'r', encoding='UTF-8')
    hexData = f.readlines()

    # Search for SI ID row and Checksum row in hex file based on signatures
    flag_sid = 0
    flag_checksum = 0
    # This flag will be set if flash size is greater than 64K, which is again
    # based on a siganture and this adds an extra row in hex file
    flag_flash_div = 0
    i = 1
    for line in hexData:
        if (line[1:13] == '020000049050'):
            flag_sid = 1
            sidLineNumber = i
        elif (line[1:13] == '020000049030'):
            flag_checksum = 1
            checksumLineNumber = i
        if (line[1:8] == '00000001'):
            lastLineNumber = i
        elif (line[1:13] == '020000040001'):
            flag_flash_div = 1
        elif (line[1:13] == '020000040002'):
            flag_flash_div = 2
        elif (line[1:13] == '020000040003'):
            flag_flash_div = 3
        i = i + 1
    
    # Verify silicon ID
    if flag_sid == 1:
        line = hexData[sidLineNumber]
        sid = int('0x' + line[13:21], 0)
    else:
        if (args.quiet == False):
            print ("ERROR: No silicon ID in " + input_file + " file.")
        exit (2)

    # Verify Checksum row
    if flag_checksum != 1:
        if (args.quiet == False):
            print ("ERROR: No Checksum row in " + input_file + " file.")
        exit (2)

    # Store metadata row index of FW APP1 based on checksumLineNumber
    fw_1_md_row_index = checksumLineNumber - 2
    # Store metadata row index of FW APP2 based on row size and fw_1_md_row_index
    fw_2_md_row_index = (fw_1_md_row_index - (device_row_size//hex_file_row_size))

    # Expected number of rows in hex file depends on the FLASH SIZE
    expectedHexFileLineCount = (device_flash_size//hex_file_row_size)

    # One bit per row info is stored in footer of hex file. If number of rows is
    # more than 0x200, one extra row is added
    if((device_flash_size//device_row_size) > 0x200):\
        expectedHexFileLineCount = expectedHexFileLineCount + 1

    #Add 9 bytes of footer
    expectedHexFileLineCount = expectedHexFileLineCount + flag_flash_div + 9

    if (len(hexData) != expectedHexFileLineCount):
        if (args.quiet == False):
            print ("ERROR: No of lines expected is not matched with the actual.");
        exit (2)

    # File is now read and stored for modification.

    # ...

def parseArguments():
    """ Parse the command line arguments """
    
    parser = argparse.ArgumentParser()

    parser.add_argument ('-i', required=True, help="Input CY HEX File.",
        dest='input_file')
    parser.add_argument ('-o', required=True, help="Output CY HEX File.",
        dest='output_file')
    parser.add_argument ('-p', required=False, help="ECC PRODUCTION Public KEY TXT File.",
        dest='prod_public_key_file')
    parser.add_argument ('-d', required=False, help="ECC DEVELOPMENT Public KEY TXT File.",
        dest='dev_public_key_file')

    parser.add_argument('-v', dest='verbose', action='store_true',
            help="optional. Turn ON verbose mode. Default is OFF.")
    parser.add_argument('-q', dest='quiet', action='store_true',
            help="optional. Turn OFF all messages. Default is ON.")

    return parser.parse_args()

    #...

#Based on existing Scripts
def calcLineChecksum (line):
    """ Calculates line checksum of hex file """
    length = int ('0x' + line[1:3], 0) + 4
    checksum = 0
    for i in range (1, 1 + length * 2, 2):
        checksum = checksum + int('0x' + line[i:i+2], 0)

    checksum = checksum & 0xFF
    checksum = 0x100 - checksum

    return checksum

    # ...

def generate_hash (app_id):
    """ Generate HASH of FW APP """
    global fw_1_md_row_index, fw_2_md_row_index

    # APP1 
    if app_id == 1:
        app_md_row_index = fw_1_md_row_index

    elif app_id == 2:
        app_md_row_index = fw_2_md_row_index

    # Use Metadata table contents to ensure that this row is actually
    # Metadata table row
    fw_entry = int.from_bytes (bytes.fromhex (str(hexData[app_md_row_index - 1][33:41])), 'little')
    fw_size = int.from_bytes (bytes.fromhex (str(hexData[app_md_row_index - 1][41:49])), 'little')
    print ("APP{} Entry: ".format(app_id) + '0x{0:08X}'.format(fw_entry))
    print ("APP{} SIZE: ".format(app_id) + '0x{0:08X}'.format(fw_size))

    # Now calculate the HASH
    sha2 = hashlib.sha256()

    # Get the start row of FW APP based on the row size and fw entry
    if (fw_entry//device_row_size) > 0x02FF:
        fw_start_row_in_hex = ((fw_entry//device_row_size) * (device_row_size//hex_file_row_size)) + 3
    if (fw_entry//device_row_size) > 0x01FF:
        fw_start_row_in_hex = ((fw_entry//device_row_size) * (device_row_size//hex_file_row_size)) + 2
    if (fw_entry//device_row_size) > 0x00FF:
        fw_start_row_in_hex = ((fw_entry//device_row_size) * (device_row_size//hex_file_row_size)) + 1
    else:
        fw_start_row_in_hex = ((fw_entry//device_row_size) * (device_row_size//hex_file_row_size))

    # Loop over the image data. Pass 64 bytes in to hash calculator till the end
    # of image
    while (fw_size != 0):
        # Read 64 bytes at a time
        line = hexData[fw_start_row_in_hex]
        # Skip over the row which divides flash in two equal sections of 64k
        # each
        if ((line[1:13] != '020000040000') and (line[1:13] != '020000040001') and (line[1:13] != '020000040002') and (line[1:13] != '020000040003')):
            sha2.update (bytes.fromhex (str(line[9:137])))
            fw_size = fw_size - hex_file_row_size
            #print ("FW Size ", fw_size)
        fw_start_row_in_hex = fw_start_row_in_hex + 1
        #print ("FW row_index ", fw_start_row_in_hex)

    #MD Row: HASH 64 bytes at a time. Metadata row can be 128 bytes or 256 bytes
    md_row_size = device_row_size//hex_file_row_size
    while (md_row_size != 0):
        sha2.update (bytes.fromhex (str(hexData[app_md_row_index - (md_row_size - 1)][9:137])))
        md_row_size = md_row_size - 1
    
    # Get message digest
    msg_digest = sha2.digest ()

    #Storing message digest (HASH) back in hex data

    # Read header of the hex data row where metadata flash row starts based on
    # row_size
    line = hexData[app_md_row_index - ((device_row_size//hex_file_row_size) - 1)][:9]
    
    #Write 32 bytes Message in hexData , one word at a time
    i = 0
    while i < 8:
        index = i*4
        line = line + '{0:08X}'.format(int.from_bytes
                (msg_digest[index:index+4], 'big'))
        i = i + 1

    #Fill rest of the MD row with zeroes
    i=0
    while i < 8:
        line = line + '00000000'
        i = i + 1

    # Calculate line checksum
    linechecksum = calcLineChecksum (line)
    if (linechecksum == 0x100):
        linechecksum = 0x00
    line = line + '{0:02X}'.format(linechecksum)
    line = line + hexData[app_md_row_index-1][139:]
    #Write back message digest row
    hexData[app_md_row_index - ((device_row_size//hex_file_row_size) - 1)] = line

    #...

# Update ECC Public Key
def update_public_key (public_key_file, is_prod):
    """ This routine updates the hex data with Public Key"""

    global fw_1_md_row_index

    # Open file in read mode
    k = open (public_key_file, 'r', encoding='UTF-8')

    #Read complete public key file
    key_data = k.read ()

    # Now store this key in hexData

    #This is the hex file format (STARTING FROM MD ROW 1):
    # MD ROW 1
    # MD ROW 2
    # PMD ROW 2
    # APP PRIORITY ROW
    # CUSTOMER ROW 1
    # CUSTOMER ROW 2
    # PUBLIC KEY ROW (First 64 bytes hold Prod Key, next 64 Hold Dev Key)

    # Get the index of Public Key row based on row size of CCG.
    public_key_row_index = (fw_1_md_row_index -
            ((5*(part_info['row_size']//hex_file_row_size)) + 1 +
            2*((part_info['row_size'] - hex_file_row_size)//hex_file_row_size)))

    # Prod or DEV?
    if (is_prod == 0):
        public_key_row_index = public_key_row_index + 1;

    line = hexData[public_key_row_index][:9]

    # This is how Pub key pem format file is decoded--
    # First line in file is ASCII TAG: ----EC PUBLIC KEY----: 26 characters, # ignore
    # Last line in file is ASCII TAG: ----END PUBLIC KEY----: 24 characters, # #ignore
    # Search for these patterns and strip them off

    if (key_data[0:27] != '-----BEGIN PUBLIC KEY-----\n'):
        print ("ERROR: Invalid PEM Format Public key file.")
        exit (2)

    key_data = key_data.lstrip ('-----BEGIN PUBLIC KEY-----\n')

    if (key_data[len(key_data) - 24 : len(key_data)] != '----END PUBLIC KEY-----\n'):
        print ("ERROR: Invalid PEM Format Public key file.")
        exit (2)

    key_data = key_data.rstrip ('----END PUBLIC KEY-----\n')

    # Next we have 36 charachtes of ASN.1 TAG: EC PARAMETERS: 36 characters, # ignore
    # Next we have public key in base64 ASCII encoded format
    # In this format , 3 hex bytes are stored as 4 ASCII characters
    # Therefore, 64 bytes of key (== 66 bytes, multiple of 3) are
    # stored as 88 ASCII characters

    # So minimum 88 + 36 characters are expected

    if (len(key_data) < 124):
        print ("ERROR: Invalid PEM Format Public key file.")
        exit (2)

    # Remove first 36 characters
    key_data = key_data[36 : len(key_data)]

    # Loop over the string and extract 88 characters

    key = ""
    counter = 0
    
    for s in key_data:
        if (s != '\n'):
            key = key + s
            counter = counter + 1

        # Break out once we have collected 88 characters
        if (counter == 88):
            break;
    
    # Ensure we pulled in 88 characters
    if (counter == 88):
        try:
            # Decode : base64 - HEX
            b = base64.b64decode (key)
        except ValueError:
            print ("ERROR: Invalid PEM Format Public key file.")
            exit (2)
    else:
        print ("ERROR: Invalid PEM Format Public key file.")
        exit (2)

    # Write 64 bytes of Key
    line = line + '{0:0=128x}'.format(int.from_bytes (b, 'big'))

    # Calculate line checksum
    linechecksum = calcLineChecksum (line)
    line = line + '{0:02X}'.format(linechecksum)
    line = line + hexData[public_key_row_index][139:]
    #Write the row back
    hexData[public_key_row_index] = line

    k.close ()

    #...
    

#Based on existing Scripts
def createHexBinFile (output_file):

    global hexData
    global checksumLineNumber

    #Open file based on extension
    fileName, fileExtension = os.path.splitext (output_file)
    if (fileExtension == '.hex'):
        t = open (output_file, 'w', encoding='UTF-8')
    #Binary
    else:
        t = open(output_file, 'wb')

    # Now write the data to file based on file type
    if (fileExtension == '.hex'):
        t.writelines (hexData)
    else:
        for line in hexData:
            #Break out when we reach the footer of hex file
            if (line[1:13] == '020000049030'):
                break
            #Skip the row which divides two flash macros
            if ((line[1:13] != '020000040001') or (line[1:13] != '020000040002') or (line[1:13] != '020000040003')):
                t.write (bytes.fromhex (str(line[9:137])))

    #DEBUG
    #if (fileExtension == '.bin'):
        #from ecdsa import SigningKey
        #sk = SigningKey.from_pem (open ("cy_priv_update.pem").read())
        #vk = sk.get_verifying_key ()
        #print (sk.to_string ())
        #print (vk.to_string ())
        #print (msg_digest_1)
        #signature = sk.sign_digest (msg_digest_1)
        #print (signature)
        #assert vk.verify_digest (signature, msg_digest_1)
        #t.write (signature)
    
    t.close ()

    # ...

if __name__ == "__main__":

    if sys.version_info[0] != 3:
        print ("Invalid Python version. Please use Python 3.x. The script was tested for 3.4.1");
        exit (1)
   
    #Parse command line arguments
    args = parseArguments()

    if (args.quiet == True):
        args.verbose = False

    filePath = os.path.dirname(os.path.realpath(sys.argv[0]))

    # Check input file
    if (os.path.exists(args.input_file) == False):
        print ("ERROR: Invalid Input File.")
        exit (2)

    fileName, fileExtension = os.path.splitext (args.input_file)
    if (fileExtension != '.hex'):
        if args.quiet == False:
            print ("ERROR: Invalid input file format.")
        exit (2)

    fileName, fileExtension = os.path.splitext (args.output_file)
    if (fileExtension != '.hex') and (fileExtension != '.bin'):
        if args.quiet == False:
            print ("ERROR: Invalid output file format.")
        exit (2)

    if (args.prod_public_key_file != None):
        fileName, fileExtension = os.path.splitext (args.prod_public_key_file)
        if (fileExtension != '.pem'):
            if args.quiet == False:
                print ("ERROR: Invalid ECC PRODUCTION Public Key File format.")
            exit (2)

    if (args.dev_public_key_file != None):
        fileName, fileExtension = os.path.splitext (args.dev_public_key_file)
        if (fileExtension != '.pem'):
            if args.quiet == False:
                print ("ERROR: Invalid ECC DEVELOPMENT Public Key File format.")
            exit (2)

    # Parse the input file
    parseInputFile (args.input_file)

    #Generate message digest and write back in hex file for both FW images
    generate_hash (1)

    # CCG1/CCG2 which have only 32KB of flash don't have support for two images
    if (device_flash_size != 0x8000):
        generate_hash (2)

    # Check is user has provided a txt file containing Publik Keys. If yes, store
    # public keys in the hex file
    if (args.prod_public_key_file != None):
        update_public_key (args.prod_public_key_file, 1)

    if (args.dev_public_key_file != None):
        update_public_key (args.dev_public_key_file, 0)

    #Generate output file
    createHexBinFile (args.output_file)
    #...
