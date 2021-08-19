import re

def change_version(m):
        return "{}.{}.{}".format(m.group(1), m.group(2), int(m.group(3))+1)

f=open('setup.py','r+')
lines=f.readlines()
f.close()
for i in range(len(lines)):
        if "version" in lines[i]:
                modify_line = lines[i]
                datapat = re.compile(r'(\d+)\.(\d+)\.(\d+)')
                lines[i] = datapat.sub(change_version, modify_line)
                f=open('setup.py','w+')
                f.writelines(lines)
                f.close()
