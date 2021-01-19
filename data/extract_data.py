import matplotlib
from matplotlib import pyplot as plt
import csv

#List for each peice of data, makes it easy for matplotlib
timestamp = []
ph_level = []
co2_level = []
temp_level = []
hum_level = []
meth_level = []


with open('practice_data.csv', encoding='utf-8-sig') as csvfile:
    reader = csv.DictReader(csvfile)
    for row in reader:
        timestamp.append(row['Timestamp'])
        ph_level.append(row['PH'])
        co2_level.append(row['CO2'])
        temp_level.append(row['Temp'])
        hum_level.append(row['Humidity'])
        meth_level.append(row['Methane'])


for char in timestamp[0]:
    if char == "/":
        temp = timestamp[0].replace("/", "-")

#Making a bar graph for PH
plt.bar(timestamp, [int(x) for x in ph_level], color='g')
plt.xlabel("Timestamp")
plt.ylabel("PH Level")
plt.title("PH Levels")
plt.xticks(size=6)
plt.savefig('Science_PH_' + temp + '.png')


#Making a bar graph for CO2
plt.bar(timestamp, [int(x) for x in co2_level], color='g')
plt.xlabel("Timestamp")
plt.ylabel("CO2 Level (PPM)")
plt.title("CO2 Levels")
plt.xticks(size=6)
plt.savefig('Science_CO2 ' + temp + '.png')

#Making a bar graph for Temp
plt.bar(timestamp, [int(x) for x in temp_level], color='g')
plt.xlabel("Timestamp")
plt.ylabel("Temperature (Celsius)")
plt.title("Temperature")
plt.xticks(size=6)
plt.savefig('Science_Temp ' + temp + '.png')

#Making a bar graph for Humidity
plt.bar(timestamp, hum_level, color='g')
plt.xlabel("Timestamp")
plt.ylabel("Humidity (%)")
plt.title("Humidity")
plt.xticks(size=6)
plt.savefig('Science_Humidity ' + temp + '.png')

#Making a bar graph for Mehtane
plt.bar(timestamp, [int(x) for x in meth_level], color='g')
plt.xlabel("Timestamp")
plt.ylabel("Methane (%)")
plt.title("Methane")
plt.xticks(size=6)
plt.savefig('Science_Meth ' + temp + '.png')
