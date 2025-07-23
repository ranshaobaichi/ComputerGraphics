# put this file in the build directory to run it

# make sure you have the Rasterizer executable in the build directory
# make sure you have make the output directory (you can modify the path if needed)

# use: chmod +x shader_runner.sh
# use: ./shader_runner.sh
for shader in texture normal phong bump displacement
do
    echo "Running shader: $shader"
    ./Rasterizer "../output/bilinear/$shader.png" $shader
    echo "$shader done"
done
wait
echo "all done"