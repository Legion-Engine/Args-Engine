
//constant values
//exit condition for poission sampling
const int K=30;
const uint RAND_MAX= 255;
//predefined values
#define maxPointsPerTri 500
#define radius 0.33f
#define cellSize (radius)/1.41421356237f
#define depth (int)ceil(1/ cellSize)
#define gridDimension  depth *depth

//seed state
uint state = 777;
float floatState=0.777f;
//returns random uint (range 0-255?)
uint Rand()
{
   state = state * 1664525 + 1013904223;
   return state >> 24;
}
//uses Rand to generate a random float (range 0-1)
float RandomValue()
{
   uint randInt =Rand();

   return randInt / (float) RAND_MAX;

}

//includes upper bound
uint RandomUpperRange(uint upper)
{
    uint r = Rand();
    return r %(upper+1);
}
//usese barycentric coordinates to sample a new point
float4 SampleTriangle(float2 coordinates,float4 a, float4 b, float4 c )
{
    return(float4)(a + coordinates.x* (b-a) + coordinates.y * (c-a));
}
float2 SampleUVs(float2 coordinates, float2 a, float2 b, float2 c)
{
    return(float2)(a + coordinates.x* (b-a) + coordinates.y * (c-a));
}
//checks validity of point for poission sampling
bool CheckPoint(float2 newPoint, float2* points, int* grid)
{
    //check if point lies within triangle range else just return false
    if(newPoint.x> 0 && newPoint.x <1 && newPoint.y>  0 && newPoint.y <1 && newPoint.x + newPoint.y < 1)
    {
        //check point validity
        //get grid cell
        int currentX = (int) newPoint.x / cellSize;
        int currentY = (int) newPoint.y / cellSize;
        //iterate neighbour cells in a radius of 2 
        int startX = max(0, currentX - 2);
        int endX = min(currentX + 2, maxPointsPerTri- 1);
        int startY = max(0, currentY - 2);
        int endY = min(currentY + 2, maxPointsPerTri-1);
        for(int x = startX; x < endX; x++)
        {
            for(int y = startY; y<endY; y++)
            {
                //get other cell, if valid, check distance is larger than radius
                int otherCellIndex = grid[x*y]-1;
                if(otherCellIndex>=0)
                {
                    float2 delta = (newPoint - points[otherCellIndex]);
                    float dist = sqrt(delta.x * delta.x + delta.y * delta.y);
                    if(dist< radius * radius) return false;
                }
            }
        }
        return true;
    }
    return false;
}
//sampling technique checkout:
//http://www.cemyuksel.com/cyCodeBase/soln/poisson_disk_sampling.html 
//to understand some more.
void PoissionSampling(__local float2* outputPoints, int samplePerTri)
{
    //init output
    int outPutIndex=0;
 
    //create cell grid
    __local int grid[maxPointsPerTri];

    __local float2 spawnPoints[maxPointsPerTri];
    //add center point as starting point
    spawnPoints[0] = (float2)(0.25f,0.25f);
    int spawnPointAmount=1;
    while(spawnPointAmount>0)
    {
       //get random point from spawn points && its position
        int index = RandomUpperRange(spawnPointAmount);
        float2 spawnCenter = spawnPoints[spawnPointAmount];

        bool isAccepteed=false;
        //try generating a valid point until k is reached
        for(int i=0; i <K; i++)
        {
            //generate offset and new point
            float angle = RandomValue() * 2 * M_PI;
            float2 direction = (float2)(sin(angle), cos(angle));
            float2 newPoint = spawnCenter + normalize(direction) * radius;
            //check point
            if(CheckPoint(newPoint,outputPoints, grid ))
            {
            outputPoints[outPutIndex] = newPoint;
            outPutIndex++;
            spawnPoints[spawnPointAmount] = newPoint;
            spawnPointAmount++;
            isAccepteed=true;

            grid[(int)(newPoint.x / cellSize) *(int)(newPoint.y/cellSize)] = outPutIndex;
            if(outPutIndex> samplePerTri) return;
            }
        }
        if(!isAccepteed)
        {
        spawnPoints[index]=0;
        spawnPointAmount--;
        }     
    }

}


float2 sampleUniformly(__local float2* output, uint samplesPerTri, uint sampleWidth)
{
    float offset = 1.0f / (float)(sampleWidth+1);
    float2 coordinates;
    int index=0;
    for(int x=0; x<sampleWidth; x++)
    {
        for(int y=0; y<sampleWidth-x; y++)
        {
           // if(x+y>sampleWidth) continue;
            coordinates=(float2)(offset*(x), offset*(y));
            output[index] = coordinates;
            index++;
        }
    }
}
constant sampler_t sampler = CLK_NORMALIZED_COORDS_FALSE | CLK_ADDRESS_CLAMP_TO_EDGE | CLK_FILTER_NEAREST;

float sampleHeight(__read_only image2d_t texture, float2 uvs, int texelSize)
{
    float r=0.0f;
    int2 newCoordinates = (int2)(uvs.x*texelSize,uvs.y*texelSize);
    float4 textureValue=read_imagef(texture, sampler, (int2)newCoordinates);
    r =textureValue.x;
    return r;
}
float4 sampleColor(__read_only image2d_t texture, float2 uvs, int texelSize)
{
    int2 newCoordinates = (int2)(uvs.x*texelSize,uvs.y*texelSize);
    float4 r =(read_imagef(texture, sampler, (int2)newCoordinates));
    return r;
}



__kernel void Main
(
    __global const float* vertices,
    __global const uint* indices,
    __global const float2* uvs,
    __global const uint* samples,
    __read_only image2d_t albedoMap,
    __read_only image2d_t normalMap,
    //const uint samplesPerTri,
//    const uint sampleWidth,
    const float normalStrength,
    const uint textureSize,
    __global float4* points,
    __global float4* colors
)
{
    //init indices and rand state
    int n=get_global_id(0)*3;
    state= get_global_id(0);
//    int resultIndex = get_global_id(0)*samplePerTri;
  
    //get vertex indices
    uint vertex1Index = indices[n];
    uint vertex2Index = indices[n+1];
    uint vertex3Index = indices[n+2];

    //get vertex values and corrosponding uvs
    //vertA
    float v1a = vertices[indices[n]*3];
    float v1b = vertices[indices[n]*3+1];
    float v1c = vertices[indices[n]*3+2];
    float4 vertA = (float4)(v1a,v1b,v1c,1.0f);
    float2 uvA =uvs[indices[n]];

    //vert2
    float v2a = vertices[indices[n+1]*3];
    float v2b = vertices[indices[n+1]*3+1];
    float v2c = vertices[indices[n+1]*3+2];
    float4 vertB = (float4)(v2a,v2b,v2c,1.0f);
    float2 uvB =uvs[indices[n+1]];

    //vert3
    float v3a = vertices[indices[n+2]*3];
    float v3b = vertices[indices[n+2]*3+1];
    float v3c = vertices[indices[n+2]*3+2];
    float4 vertC = (float4)(v3a,v3b,v3c,1.0f);
    float2 uvC =uvs[indices[n+2]];

    //calculate current sample count
    float lengthA = length(vertC-vertA);
    float lengthB = length(vertB-vertA);
    float lengthC = length(vertC-vertB);
    float size = lengthA+lengthB+lengthC;

    uint newSampleCount = samples[get_global_id(0)];
    uint resultIndex=0;
    //calculate new result ID, accumulate previous sample counts
    for(int i=0; i< get_global_id(0); i++)
    {
        resultIndex+= samples[i];
    }

    //calculate the sample width for the newly generated sample count
    uint currentIt=0;
    uint sum =0;
    while(sum< newSampleCount)
    {
        currentIt++;
        sum +=currentIt;
    }
    uint sampleWidth=currentIt;


    //generate normal && scale by strength
    float4 normal = normalize(cross(vertB-vertA,vertC-vertA));
    normal*=normalStrength;
    
    //generate samples
    __local float2 uniformOutput[maxPointsPerTri];
    sampleUniformly(uniformOutput,newSampleCount,sampleWidth);
    //__local float2 poissonOutput[maxPointsPerTri];
    //PoissionSampling(poissonOutput,samplePerTri);


    //store generated samples
    for(int i =0; i <newSampleCount; i++)
    {
        int index= resultIndex + i;
        //sample point position
        float4 newPoint =SampleTriangle(uniformOutput[i],vertA,vertB,vertC);

        //get uvs
        float2 uvCoordinates = SampleUVs(uniformOutput[i],uvA,uvB,uvC);
        //sample height based on uvs
        float heightOffset = sampleHeight(normalMap,uvCoordinates ,textureSize);
        float4 Color = sampleColor(albedoMap,uvCoordinates ,textureSize);
      //  Color = (float4)(uvCoordinates.x, uvCoordinates.y, 0, 1);
        //Color = (float4)(1,0,0,1);
        //scale normal by the height & add it to the point
        newPoint+= normal*heightOffset;
        colors[index]=Color;
        points[index]=newPoint;
    }
}

