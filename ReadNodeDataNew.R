###############################  read soil sensor, reference and status data  ###############################
#
# read all files into one large data frame
# install.packages("stringr", repos='http://cran.us.r-project.org')
# previous analysis method read files three times:  for TDC data, temp data, vBatt data and timestamp
# here all are read in one pass

library(stringr)
library(magrittr)
library(pracma)

remove(list = ls())

ref_date = "1970/1/1" #for unix timestamp

setwd(
  "C:/Users/rjwil/Documents/Computer/Arduino/LC_Capacitance_measurement/Soil meters/SoilMoistureHighFrequencyCapacitanceMeasurement/data/"
)

subdirName = "181225"
FileNames = Sys.glob(file.path(subdirName, "SD_??_??.*"))

# prepare regular expression
regexp <- "\\d+\\.*\\d*"


###############################  dayofyear   ###############################

dayofyear = function(timedata, ref) {
  #days since 1/1/15 0000h, i.e.unix timestamp 1420070400
  # '0' for first day of year
  # this function uses the next one
  # posix returns a list of time attributes, month, day, yday etc. Here use yday
  return(posixtime(timedata, ref)[[8]])
  
}

###############################  hourofday   ###############################

hourofday = function(timedata, ref) {
  #hours since 1/1/15 0000h, i.e.unix timestamp 1420070400
  # '0' for first day of year
  # this function uses the next one
  # posix returns a list of time attributes, month, day, yday etc. Here use yday
  return(posixtime(timedata, ref)[[3]])
  
}
###############################  hourofyear   ###############################

hourofyear = function(timedata, ref) {
  #hours since 1/1/15 0000h, i.e.unix timestamp 1420070400
  # '0' for first day of year
  # posix returns a list of time attributes, month, day, yday etc. Here use yday
  return(posixtime(timedata, ref)[[3]] + 24 * posixtime(timedata, ref)[[8]])
  
}

###############################  hourofyear   ###############################

time_interval_of_year = function(timedata, ref, interval) {
  #intervals since 1/1/15 0000h, i.e.unix timestamp 1420070400
  # '0' for first interval of year
  # posix returns a list of time attributes, month, day, yday etc. Here use yday
  return((posixtime(timedata, ref)[[2]] +
            60 * (
              posixtime(timedata, ref)[[3]] + 24 * posixtime(timedata, ref)[[8]]
            )) %/% interval)
  
}

###############################  posix_unixtime   ###############################
posixtime = function(timestamp, reference_date) {
  return(as.POSIXlt(timestamp, origin = reference_date))
}

###############################  unix_timestring   ###############################

unixtime = function(charstring, reference_date) {
  # returns unixtime as seconds since reference_date
  # charstring format example: "7/13/2015 19:52:25"
  test = strptime(charstring, "%m/%d/%Y %H:%M:%S", tz = "GMT")
  return (as.integer(as.POSIXct(test, origin = reference_date, tz = "GMT")))
}

###############################  filename_unixtime   ###############################

#function to create string for filename from UNIX time
# convert unix time to format yy-mm-dd
filename_unixtime = function(timestamp) {
  fname = as.character(posixtime(timestamp))
  substr(fname, 5, 5) = "_"
  substr(fname, 8, 8) = "_"
  substr(fname, 1, 10)
}

##################################### read in soil sensor data #####################################
rowLimit = 500000 # not sure whether needed

mycols <- rep("NULL", 11)
mycols[c(2, 3, 4, 9, 10)] <- NA


read_data =  data.frame(
  numeric(),
  numeric(),
  numeric(),
  numeric(),
  numeric(),
  numeric(),
  numeric(),
  numeric(),
  numeric(),
  numeric(),
  numeric()
)

for (i in 1:length(FileNames))
{
  # NULL in mycols defines which columns to omit
  # note need to name all of them
  new_data = read.csv(
    FileNames[i],
    header = FALSE,
    sep = ",",
    quote = "\"",
    dec = ".",
    skip = 3,
    stringsAsFactors = FALSE,
    col.names = c(
      "v1",
      "Time",
      "Node",
      "Count",
      "v5",
      "v6",
      "v7",
      "v8",
      "PeriodCoarse",
      "PeriodFine",
      "v11"
    ),
    colClasses = mycols,
    fill = TRUE,
    blank.lines.skip = TRUE,
    nrows = rowLimit
  )
  
  # omit rows with N/A values
  new_data = na.omit(new_data)
  
  read_data = rbind(read_data, new_data)
}

for (i in 1:5)
{
  read_data[,i] = as.numeric(read_data[,i])
}

soilSensor = subset.data.frame(read_data, read_data$Node == 28)
newSoilSensor = subset.data.frame(read_data, read_data$Node%%16 == 13)
refSensor = subset.data.frame(read_data, read_data$Node == 12)



##################################### read in garage temp data #####################################


mycols <- rep("NULL", 11)
mycols[c(2, 3, 4, 6)] <- NA


read_data =  data.frame(
  numeric(),
  numeric(),
  numeric(),
  numeric(),
  numeric(),
  numeric(),
  numeric(),
  numeric(),
  numeric(),
  numeric(),
  numeric()
)

for (i in 1:length(FileNames))
{
  # NULL in mycols defines which columns to omit
  # note need to name all of them
  new_data = read.csv(
    FileNames[i],
    header = FALSE,
    sep = ",",
    quote = "\"",
    dec = ".",
    skip = 1,
    stringsAsFactors = FALSE,
    col.names = c(
      "v1",
      "Time",
      "Node",
      "Count",
      "v5",
      "Temp",
      "v7",
      "v8",
      "v9",
      "v10",
      "v11"
    ),
    colClasses = mycols,
    fill = TRUE,
    blank.lines.skip = TRUE,
    nrows = rowLimit
  )
  
  # omit rows with N/A values
  new_data = na.omit(new_data)
  
  read_data = rbind(read_data, new_data)
}

for (i in 1:4)
{
  read_data[,i] = as.numeric(read_data[,i])
}

garageSensor = subset.data.frame(read_data, (read_data$Node %% 16) == 7)


##################################### read in status data #####################################

read_data =  data.frame(
  numeric(),
  numeric(),
  numeric(),
  numeric(),
  numeric(),
  numeric(),
  numeric(),
  numeric(),
  numeric(),
  numeric(),
  numeric()
)

mycols <- rep("NULL", 11)
mycols[c(2, 3, 4, 5, 6, 7)] <- NA

for (i in 1:length(FileNames))
{
  # NULL in mycols defines which columns to omit
  # note need to name all of them
  new_data = read.csv(
    FileNames[i],
    header = FALSE,
    sep = ",",
    quote = "\"",
    dec = ".",
    skip = 1,
    stringsAsFactors = FALSE,
    col.names = c(
      "v1",
      "Time",
      "Node",
      "Count",
      "Temp",
      "Vbatt",
      "millisec",
      "v8",
      "v9",
      "v10",
      "v11"
    ),
    colClasses = mycols,
    fill = TRUE,
    blank.lines.skip = TRUE,
    nrows = rowLimit
  )
  
  # omit rows with N/A values
  new_data = na.omit(new_data)
  
  read_data = rbind(read_data, new_data)
}

for (i in 1:6)
{
  read_data[,i] = as.numeric(read_data[,i])
}

status = subset.data.frame(read_data, read_data$Node == 44)
status$Temp = status$Temp / 16

##################################### read in newSoilSense data #####################################

read_data =  data.frame(
  numeric(),
  numeric(),
  numeric(),
  numeric(),
  numeric(),
  numeric(),
  numeric(),
  numeric(),
  numeric(),
  numeric(),
  numeric()
)

mycols <- rep("NULL", 11)
mycols[c(2, 3, 4, 5, 6, 7, 8)] <- NA

for (i in 1:length(FileNames))
{
  # NULL in mycols defines which columns to omit
  # note need to name all of them
  new_data = read.csv(
    FileNames[i],
    header = FALSE,
    sep = ",",
    quote = "\"",
    dec = ".",
    skip = 3,
    stringsAsFactors = FALSE,
    col.names = c(
      "v1",
      "Time",
      "Node",
      "Count",
      "Temp",
      "Vbatt",
      "millisec",
      "period",
      "v9",
      "v10",
      "v11"
    ),
    colClasses = mycols,
    fill = TRUE,
    blank.lines.skip = TRUE,
    nrows = rowLimit
  )
  
  # omit rows with N/A values
  new_data = na.omit(new_data)
  
  read_data = rbind(read_data, new_data)
}

for (i in 1:7)
{
  read_data[,i] = as.numeric(read_data[,i])
}

newSoilSensor = subset.data.frame(read_data, (read_data$Node %% 16) == 13)

newSoilSensor$Temp = newSoilSensor$Temp / 16


###############################  SET START AND END TIME FOR ANALYSIS  ###############################
# startTime = unixtime("11/26/2018 06:00:00", ref_date)
# endTime = unixtime("12/01/2018 06:00:00", ref_date)
# startTime = unixtime("12/05/2018 06:00:00", ref_date)
# endTime = unixtime("12/07/2018 16:00:00", ref_date)
# startTime = unixtime("12/08/2018 21:00:00", ref_date)
# endTime = unixtime("12/09/2018 04:00:00", ref_date)
startTime = unixtime("12/24/2018 18:30:00", ref_date)
endTime = unixtime("12/25/2018 12:30:00", ref_date)
###############################  plot frequency of soil sensor node 28 in a user defined period  ###############################
show_data = subset.data.frame(soilSensor,
                              (soilSensor$Time >= startTime) & (soilSensor$Time < endTime))

divideby = 2^17
Frequency = divideby / show_data$PeriodCoarse

ymin = 38
ymax = 44
yint = .5
# ymin = min(Frequency)
# ymax = max(Frequency)
# yint = 1
timeInt = 3600
lineWidth = .5
palette = c("blue","green","red","violet","cyan","pink","springgreen","darkviolet","orange","brown","black","darkolivegreen4","darkgray",	"gold",	"plum3",	"thistle4", "pink", "azure")

windows(18, 6)

plot(
  posixtime(show_data$Time, ref_date),
  Frequency,
  type = "p",
  # xlim = c(xmin, xmax),
  ylim = c(ymin, ymax),
  col = palette[1],
  pch = 2,
  cex = .3,
  lwd = lineWidth
)

yvals <- seq(ymin,ymax,by=yint)
abline(h=yvals,v=NULL,col="gray",lty=3)

xvals <- seq(startTime,endTime, by=timeInt)
abline(v=xvals,h=NULL,col="gray",lty=3)


###############################  superpose coarse frequency of soil sensor  ###############################
show_data = subset.data.frame(soilSensor,
                              (soilSensor$Time >= startTime) & (soilSensor$Time < endTime))

frequencyCoarse = divideby / show_data$PeriodCoarse

lineWidth = .5
palette = c("blue","green","red","violet","cyan","pink","springgreen","darkviolet","orange","brown","black","darkolivegreen4","darkgray",	"gold",	"plum3",	"thistle4", "pink", "azure")

points(
  posixtime(show_data$Time, ref_date),
  frequencyCoarse,
  type = "p",
  col = palette[4],
  pch = 3,
  cex = .3,
  lwd = lineWidth
)


###############################  plot frequency of reference sensor node 12  ###############################

ymin = 64.15
ymax = 64.35
yint = .05
show_data = subset.data.frame(refSensor,
                              (refSensor$Time >= startTime) &
                                (refSensor$Time < endTime))
# select to reference data on soilSensor measurement
# points(
#   posixtime(show_data$Time, ref_date),
#   ((divideby/show_data$PeriodFine) - 64.15) * 10 + 39,
windows(18,6)

plot(
  posixtime(show_data$Time, ref_date),
  divideby / show_data$PeriodCoarse,
  type = "b",
  ylim = c(ymin, ymax),
  col = palette[2],
  pch = 3,
  cex = 1,
  lwd = lineWidth
)

yvals <- seq(ymin,ymax,by=yint)
abline(h=yvals,v=NULL,col="gray",lty=3)

xvals <- seq(startTime,endTime, by=timeInt)
abline(v=xvals,h=NULL,col="gray",lty=3)

###############################  superpose coarse frequency of reference sensor  ###############################
show_data = subset.data.frame(refSensor,
                              (refSensor$Time >= startTime) &
                                (refSensor$Time < endTime))

frequencyCoarse = divideby / show_data$PeriodCoarse

timeInt = 3600 * 1
lineWidth = .5
divideby = 2^17
palette = c("blue","green","red","violet","cyan","pink","springgreen","darkviolet","orange","brown","black","darkolivegreen4","darkgray",	"gold",	"plum3",	"thistle4", "pink", "azure")

points(
  posixtime(show_data$Time, ref_date),
  frequencyCoarse,
  type = "b",
  col = palette[4],
  pch = 4,
  cex = 1,
  lwd = lineWidth
)


###############################  superpose temperature  ###############################

show_data = subset.data.frame(status,
                              (status$Time >= startTime) & (status$Time < endTime))
points(
  posixtime(show_data$Time, ref_date),
  ((show_data$Temp / 100) + 64.05),
  type = "p",
  col = palette[3],
  pch = 3,
  cex = .4,
  lwd = lineWidth
)


###############################  plot frequency of new soil sensor node 13  ###############################
show_data = subset.data.frame(newSoilSensor,
                              (newSoilSensor$Time >= startTime) & (newSoilSensor$Time < endTime))

divideby = 2^17
Frequency = divideby / show_data$period

ymin = 113
ymax = 117
yint = .5
# ymin = min(Frequency)
# ymax = max(Frequency)
# yint = 1
timeInt = 3600
lineWidth = .5
palette = c("blue","green","red","violet","cyan","pink","springgreen","darkviolet","orange","brown","black","darkolivegreen4","darkgray",	"gold",	"plum3",	"thistle4", "pink", "azure")

windows(18, 6)

plot(
  posixtime(show_data$Time, ref_date),
  Frequency,
  type = "p",
  # xlim = c(xmin, xmax),
  ylim = c(ymin, ymax),
  col = palette[1],
  pch = 2,
  cex = .3,
  lwd = lineWidth
)

yvals <- seq(ymin,ymax,by=yint)
abline(h=yvals,v=NULL,col="gray",lty=3)

xvals <- seq(startTime,endTime, by=timeInt)
abline(v=xvals,h=NULL,col="gray",lty=3)


###############################  plot temperature node 44  ###############################

ymin = 15
ymax = 25
yint = 1
show_data = subset.data.frame(status,
                              (status$Time >= startTime) & (status$Time < endTime))
windows(18,6)
plot(
  posixtime(show_data$Time, ref_date),
  show_data$Temp,
  type = "p",
  # xlim = c(xmin, xmax),
  ylim = c(ymin, ymax),
  col = palette[3],
  pch = 3,
  cex = .4,
  lwd = lineWidth
)

yvals <- seq(ymin,ymax,by=yint)
abline(h=yvals,v=NULL,col="gray",lty=3)

xvals <- seq(startTime,endTime, by=timeInt)
abline(v=xvals,h=NULL,col="gray",lty=3)
###############################  superpose temperature node 13  ###############################

show_data = subset.data.frame(newSoilSensor,
                              (status$Time >= startTime) & (status$Time < endTime))
points(
  posixtime(show_data$Time, ref_date),
  show_data$Temp,
  type = "p",
  # xlim = c(xmin, xmax),
  ylim = c(ymin, ymax),
  col = palette[4],
  pch = 3,
  cex = .4,
  lwd = lineWidth
)



###############################  plot Vbatt  ###############################

ymin = 4500
ymax = 5000
yint = 100
show_data = subset.data.frame(status,
                              (status$Time >= startTime) & (status$Time < endTime))

windows(8,8)
plot(
  posixtime(show_data$Time, ref_date),
  show_data$Vbatt,
  type = "p",
  ylim = c(ymin, ymax),
  col = palette[3],
  pch = 3,
  cex = .4,
  lwd = lineWidth
)

yvals <- seq(ymin,ymax,by=yint)
abline(h=yvals,v=NULL,col="gray",lty=3)

xvals <- seq(startTime,endTime, by=timeInt)
abline(v=xvals,h=NULL,col="gray",lty=3)

###############################  plot millisec from status report  ###############################

ymin = 0
yint = 3.6e6
ymax = 12 * yint
show_data = subset.data.frame(status,
                              (status$Time >= startTime) & (status$Time < endTime))

windows(8,8)
plot(
  posixtime(show_data$Time, ref_date),
  show_data$millisec,
  type = "p",
  ylim = c(ymin, ymax),
  col = palette[3],
  pch = 3,
  cex = .4,
  lwd = lineWidth
)

yvals <- seq(ymin,ymax,by=yint)
abline(h=yvals,v=NULL,col="gray",lty=3)

xvals <- seq(startTime,endTime, by=timeInt)
abline(v=xvals,h=NULL,col="gray",lty=3)

print((max(show_data$millisec) - min(show_data$millisec)) / (max(show_data$Time) - min(show_data$Time)))
###############################  plot temperature of garage ###############################

ymin = 0
ymax = 30
yint = 1
show_data = subset.data.frame(garageSensor,
                              (garageSensor$Time >= startTime) & (garageSensor$Time < endTime))
windows(8,8)
plot(
  posixtime(show_data$Time, ref_date),
  # ((show_data$Temp / 20) + 38.5),
  show_data$Temp,
  type = "p",
  # xlim = c(xmin, xmax),
  ylim = c(ymin, ymax),
  col = palette[3],
  pch = 3,
  cex = .4,
  lwd = lineWidth
)

yvals <- seq(ymin,ymax,by=yint)
abline(h=yvals,v=NULL,col="gray",lty=3)

xvals <- seq(startTime,endTime, by=timeInt)
abline(v=xvals,h=NULL,col="gray",lty=3)

###############################  plot delta f / f of soil sensor  ###############################
ymin = -5e-2
ymax = 5e-2
yint = .005
timeInt = 3600 * 2
lineWidth = .5
palette = c("blue","green","red","violet","cyan","pink","springgreen","darkviolet","orange","brown","black","darkolivegreen4","darkgray",	"gold",	"plum3",	"thistle4", "pink", "azure")

show_data = subset.data.frame(soilSensor,
                              (soilSensor$Time >= startTime) & (soilSensor$Time < endTime))

Frequency = divideby / show_data$PeriodFine
averageFrequency = mean(Frequency)
windows(18, 6)

plot(
  posixtime(show_data$Time, ref_date),
  (Frequency - averageFrequency),
  # (Frequency / averageFrequency) - 1,
  type = "p",
  # xlim = c(xmin, xmax),
  ylim = c(ymin, ymax),
  col = palette[1],
  pch = 3,
  cex = .4,
  lwd = lineWidth
)

yvals <- seq(ymin,ymax,by=yint)
abline(h=yvals,v=NULL,col="gray",lty=3)

xvals <- seq(startTime,endTime, by=timeInt)
abline(v=xvals,h=NULL,col="gray",lty=3)

###############################  calculate cLadder OLD  ###############################
lengthLadder = 100 # cm
LLadder = 15.133e-9 # H/cm
LTank = 787e-9 + LLadder * lengthLadder
C1 = 22e-12 + 5e-12 # nominal capacitance + parasitic + input and output capacitance of BC547B in Farad 
#where capacitors C1, C2 and C3 in resonant circuit
#C3 is the capacitance of the soil sensor and in series with L
#C1=C2
Cparallel = 2e-12 # in parallel with ladder


CLadder = (1 / ((2 * pi * Frequency * 1e6)^2 * LTank - 2 / C1)) - Cparallel
#  CLadder = .074e-12 F/cm from http://www.mantaro.com/resources/impedance-calculator.htm

###############################  calculate eps medium  ###############################

epsWater = 80
epsMatrix = 1.2
#https://inis.iaea.org/collection/NCLCollectionStore/_Public/27/040/27040410.pdf
#cites values 3.5 (sand 2.65% H2O), 38 (Toledo bend 25.6% H2O) @ f=100MHz
#https://www.hindawi.com/journals/js/2016/2827890/


###############################  LTSpice simulation of frequency vs unit capacitance of ladder  ###############################

capUnitLen = c(
  1.0000E-14,
  1.1220E-14,
  1.2589E-14,
  1.4125E-14,
  1.5849E-14,
  1.7783E-14,
  1.9953E-14,
  2.2387E-14,
  2.5119E-14,
  2.8184E-14,
  3.1623E-14,
  3.5481E-14,
  3.9811E-14,
  4.4668E-14,
  5.0119E-14,
  5.6234E-14,
  6.3096E-14,
  7.0795E-14,
  7.9433E-14,
  8.9125E-14,
  1.0000E-13,
  1.1220E-13,
  1.2589E-13,
  1.4125E-13,
  1.5849E-13,
  1.7783E-13,
  1.9953E-13,
  2.2387E-13,
  2.5119E-13,
  2.8184E-13,
  3.1623E-13,
  3.5481E-13,
  3.9811E-13,
  4.4668E-13,
  5.0119E-13,
  5.6234E-13,
  6.3096E-13,
  7.0795E-13,
  7.9433E-13,
  8.9125E-13,
  1.0000E-12
  )
# capUnitLen from LTSpice simulation 'Clapp Circuit 181208 tidy'

calcFrequency = c(
  1.0676E+08,
  1.0425E+08,
  1.0159E+08,
  9.8881E+07,
  9.7678E+07,
  9.6602E+07,
  9.0127E+07,
  8.7158E+07,
  8.4136E+07,
  8.1144E+07,
  8.1742E+07,
  7.5703E+07,
  7.2458E+07,
  6.9713E+07,
  6.7064E+07,
  6.4542E+07,
  6.3312E+07,
  6.1522E+07,
  5.7502E+07,
  5.7360E+07,
  5.3405E+07,
  5.1472E+07,
  4.9740E+07,
  4.7780E+07,
  4.6203E+07,
  4.4597E+07,
  4.3594E+07,
  4.3248E+07,
  4.1926E+07,
  3.9991E+07,
  3.8262E+07,
  3.8479E+07,
  3.4791E+07,
  3.4387E+07,
  3.2577E+07,
  3.0988E+07,
  2.9144E+07,
  2.7973E+07,
  3.3224E+07,
  5.2333E+07,
  3.0576E+07
) / 1e6
# calcFrequency from LTSpice simulation 'Clapp Circuit 181208 tidy'

remove = c(11,32,39:41)
# need to remove values leading to 1:many mapping 
capUnitLen = capUnitLen[-remove]
calcFrequency =calcFrequency[-remove]
df = data.frame(capUnitLen,calcFrequency)
df = df[order(df$calcFrequency),]
plot(df$capUnitLen,df$calcFrequency)

CLadder = cubicspline(df$calcFrequency, df$capUnitLen, Frequency)

###############################  FEA simulation of unit capacitance of ladder vs dielectric of surrounding medium  ###############################
# the response to the external medium is reduced by the presence of dielectric around the conductors
# from FEA calculations assuming eps insulator = 2.2 (polyethylene) and thickness 0.65mm
cap = c(0,
        9,
        17,
        32.8,
        47.5,
        61.3,
        73,
        80
) * 1e-14 # cap per cm results from FEA calculation
eps = c(0,
        1,
        2,
        5,
        10,
        20,
        40,
        80
)  # eps medium dependent parameter in FEA calculation
epsMedium = cubicspline(cap, eps, CLadder)
# eps is the dielectric function of the medium set in the FEA model used to calculate cap, the capacitance of the ladder
# epsMedium is the measured average dielectric function of the medium surrounding the sensor


###############################  plot eps medium  ###############################

ymin = 4.4
ymax = 4.8
yint = .05
timeInt = 3600 * 2
lineWidth = .5

windows(8, 8)

plot(
  posixtime(show_data$Time, ref_date),
  epsMedium,
  type = "p",
  # xlim = c(xmin, xmax),
  ylim = c(ymin, ymax),
  col = palette[1],
  pch = 3,
  cex = .4,
  lwd = lineWidth
)

yvals <- seq(ymin,ymax,by=yint)
abline(h=yvals,v=NULL,col="gray",lty=3)

xvals <- seq(startTime,endTime, by=timeInt)
abline(v=xvals,h=NULL,col="gray",lty=3)

###############################  calculate volume fraction  ###############################
vWater = (epsMedium - epsMatrix) / (epsWater - epsMatrix) * 100 # in percent

ymin = 0
ymax = max(ceiling(vWater))
yint = 1
timeInt = 3600 * 2
lineWidth = .5

windows(8, 8)

plot(
  posixtime(show_data$Time, ref_date),
  vWater,
  type = "p",
  # xlim = c(xmin, xmax),
  ylim = c(ymin, ymax),
  col = palette[1],
  pch = 3,
  cex = .4,
  lwd = lineWidth
)

yvals <- seq(ymin,ymax,by=yint)
abline(h=yvals,v=NULL,col="gray",lty=3)

xvals <- seq(startTime,endTime, by=timeInt)
abline(v=xvals,h=NULL,col="gray",lty=3)



###############################  plot Tnode 5  ###############################

tempSensor = subset.data.frame(read_data, read_data$Node == 5)
show_data = subset.data.frame(tempSensor,
                              (tempSensor$Time >= startTime) & (tempSensor$Time < endTime))


ymin = 0
ymax = 25
yint = 5
# ymin = min(Frequency)
# ymax = max(Frequency)
# yint = 1
timeInt = 3600 * 2
lineWidth = .5
palette = c("blue","green","red","violet","cyan","pink","springgreen","darkviolet","orange","brown","black","darkolivegreen4","darkgray",	"gold",	"plum3",	"thistle4", "pink", "azure")

windows(18, 6)

plot(
  posixtime(show_data$Time, ref_date),
  show_data$Vbatt/100,
  type = "p",
  # xlim = c(xmin, xmax),
  ylim = c(ymin, ymax),
  col = palette[1],
  pch = 3,
  cex = .4,
  lwd = lineWidth
)

yvals <- seq(ymin,ymax,by=yint)
abline(h=yvals,v=NULL,col="gray",lty=3)

xvals <- seq(startTime,endTime, by=timeInt)
abline(v=xvals,h=NULL,col="gray",lty=3)

###############################  end  ###############################

