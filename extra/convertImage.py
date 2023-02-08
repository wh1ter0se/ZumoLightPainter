import csv

output = '{\n'
for line in open('husky2.txt', 'r'):
    output += '{'
    for word in line.split():
        output += f'\'{str.upper(word)}\','
    output = output[:-1]
    output += '}\n'
output += '}'

print(output)
f = open("husky2_conv.txt", "a")
f.write(output)
f.close()