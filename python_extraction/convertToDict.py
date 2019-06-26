MyDict= {}

with open('/home/mayadi/python/at86rf215.txt','r') as f:
	data = f.read()
	for line in data.splitlines():
		if '#define' in line:
			reg=line.split('#define')[1].strip()
			value=reg.split(' ')[0].strip()
			key=reg.split(value)[1].strip()
			if key[0:3] == "(0x":
				key =key[1:-1]
				while len(key)!=6:
					key=key[0:2]+'0'+key[2:len(key)]
#				print(value)
				MyDict[key] =value

#for key, value in MyDict.items():
#	print( key, value )
print (MyDict)
