
if True:
    f = open("twenty_dict.txt", "r")
    f_write = open("../twenty_watt_dict.py", "w")
    dictionaryName = "ten_watt_dict"
    print(dictionaryName + " = {")
    f_write.write(dictionaryName + " = {")
    for x in f:
        line = x.strip().split(",")
        key = line[0]
        val = line[1]
        print("\t" + key + ": " + val + ",")
        f_write.write("\t" + key + ": " + val + ",")
    print("}")
    f_write.write("}")
    f_write.close()

if False:
    f = open("hundredthDict.txt", "r")
    arrayName = "hundredthDict"
    dataType = "double"
    print("const" + dataType + arrayName + " = {")
    for x in f:
        line = x.strip().split(",")
        val = line[1]
        print("\t" + val + ",")
    print("}")