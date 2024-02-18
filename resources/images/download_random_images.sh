
# bulk download random images with predefined keywords and size from loremflickr.com to current directory

# config
KEYWORDS="pedestrians"   # search keywords comma separated
WIDTH=640 # 320               # image width
HEIGHT=640 # 320              # image high
COUNT=100               # image count
MAXTRIALS=3             # max errors until stop

find . -name "*.jpg" -type f -delete

i=0
k=0
while [ $i -lt $COUNT ]
do
    wget -q --show-progress "https://loremflickr.com/g/${WIDTH}/${HEIGHT}/${KEYWORDS}/all" -O "${i}.jpg" # loremflickr.com returns random image
    if [ "$?" -ne 0 ]; then
        k=$[$k+1]
        if [ "$k" == "$MAXTRIALS" ]; then
            echo "too many errors. abort..."
            exit 1
        fi
        continue;
    fi
    k=0
    i=$[$i+1]
    sleep 0.5 # wait after each request, do not send too many in short time
done 
