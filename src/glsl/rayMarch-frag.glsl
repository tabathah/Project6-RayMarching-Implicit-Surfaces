
#define MAX_GEOMETRY_COUNT 100
#define MAX_ITER 80

/* This is how I'm packing the data
struct geometry_t {
    vec3 position;
    float type;
};
*/
uniform vec4 u_buffer[MAX_GEOMETRY_COUNT];
uniform int u_count;
uniform vec3 cameraPos;
uniform mat4 cameraTransform;
uniform float tanAlpha;
uniform float aspRatio;

uniform int time;

varying vec2 f_uv;

//returns direction of ray from camera to this fragment
vec3 createRay()
{
	vec3 F = normalize(vec3(cameraTransform * vec4(0.0, 0.0, -1.0, 0.0)));
	vec3 R = normalize(cross(F, vec3(0.0, 1.0, 0.0)));
	vec3 U = normalize(cross(R, F));
	vec3 ref = cameraPos + 0.1*F;
	float len = length(ref - cameraPos);
	vec3 p = ref + ((2.0*f_uv.x-1.0) * (R*len*aspRatio*tanAlpha)) + ((2.0*f_uv.y-1.0) * (U*len*tanAlpha));
    return normalize(p-cameraPos);
}

//returns transformed point based on rotation and translation matrix of shape
vec3 transform(vec3 point, mat4 trans)
{
	//columns of the rotation matrix transpose
	vec3 col1 = vec3(trans[0][0], trans[1][0], trans[2][0]);
	vec3 col2 = vec3(trans[0][1], trans[1][1], trans[2][1]);
	vec3 col3 = vec3(trans[0][2], trans[1][2], trans[2][2]);

	mat3 rotTranspose = mat3(col1, col2, col3);

	vec3 col4 = -1.0*rotTranspose*vec3(trans[3]);

	mat4 newTrans = mat4(vec4(col1, 0.0), vec4(col2, 0.0), vec4(col3, 0.0), vec4(col4, 1.0));

	return vec3(newTrans * vec4(point, 1.0));
}

//SDFS:

float sphereSDF(vec3 point, mat4 trans, float radius) {
	vec3 p = transform(point, trans); //puts point in local object space
	return length(p) - radius;
}

//diagonal is the vector from the center of the box to the first quadrant corner
float boxSDF(vec3 point, mat4 trans, vec3 diagonal) {
	vec3 p = transform(point, trans); //puts point in local object space
	vec3 d = abs(p) - diagonal;
  	return min(max(d.x,max(d.y,d.z)),0.0) + length(max(d,0.0));
}

//first value in param is radius, second value is, third value is the height
float coneSDF(vec3 point, mat4 trans, vec3 param)
{
    vec3 p = transform(point, trans); //puts point in local object space
    vec2 q = vec2( length(p.xz), p.y );
    vec2 v = vec2( param.z*param.y/param.x, -param.z );
    vec2 w = v - q;
    vec2 vv = vec2( dot(v,v), v.x*v.x );
    vec2 qv = vec2( dot(v,w), v.x*w.x );
    vec2 d = max(qv,0.0)*qv/vv;
    return sqrt( dot(w,w) - max(d.x,d.y) ) * sign(max(q.y*v.x-q.x*v.y,w.y));
}

//first value in param is inner radius, second value is the thickness
float torusSDF(vec3 point, mat4 trans, vec2 param) {
	vec3 p = transform(point, trans); //puts point in local object space
	vec2 q = vec2(length(p.xz)-param.x,p.y);
  	return length(q)-param.y;
}

//first value in param is radius, second value is the height
float cylinderSDF(vec3 point, mat4 trans, vec2 param) {
	vec3 p = transform(point, trans); //puts point in local object space
	vec2 d = abs(vec2(length(p.xz),p.y)) - param;
  	return min(max(d.x,d.y),0.0) + length(max(d,0.0));
}

//Operators:

float intersection(float d1, float d2)
{
    return max(d1,d2);
}

float subtraction( float d1, float d2 )
{
    return max(-d1,d2);
}

float un(float d1, float d2)
{
    return min(d1,d2);
}

float getMod(int num1, int num2)
{
	int div = num1/num2;
	return float(num1 - div*num2);
}

//returns the new step size for the sphere marching, holds the info for the scene
float sphereTrace(vec3 point)
{
	//bounding volume for body
    float bbb = boxSDF(point, mat4(1.0), vec3(2.0,2.0,1.5));    

    float fullbody;
    if(bbb < .015)
    {
    	//body cube of robot, centered at origin, scaled
    	float body = boxSDF(point, mat4(1.0), vec3(2.0,2.0,1.0));

    	//sphere buttons on robots chest
    	mat4 b1Mat = mat4(1.0); b1Mat[3][0] = -1.5; b1Mat[3][1] = 1.5; b1Mat[3][2] = 1.0;
    	float b1 = sphereSDF(point, b1Mat, 0.25);

    	mat4 b2Mat = mat4(1.0); b2Mat[3][0] = -0.75; b2Mat[3][1] = 1.5; b2Mat[3][2] = 1.0;
    	float b2 = sphereSDF(point, b2Mat, 0.25);

    	mat4 b3Mat = mat4(1.0); b3Mat[3][0] = -1.5; b3Mat[3][1] = 0.75; b3Mat[3][2] = 1.0;
    	float b3 = sphereSDF(point, b3Mat, 0.25);

    	mat4 b4Mat = mat4(1.0); b4Mat[3][0] = -0.75; b4Mat[3][1] = 0.75; b4Mat[3][2] = 1.0;
    	float b4 = sphereSDF(point, b4Mat, 0.25);

    	fullbody = un(body, un(b1, un(b2, un(b3, b4))));
    }
    else
    {
    	fullbody = bbb;
    }



   	float animHeight; //height change for animation
   	float mod = getMod(time, 120);
   	if(mod < 61.0) {animHeight = mod/60.0;}
   	else {animHeight = (120.0-mod)/60.0;}



   	//bounding volume for neck
   	mat4 bbnMat = mat4(1.0); bbnMat[3][1] = 2.0 + animHeight;
    float bbn = boxSDF(point, bbnMat, vec3(1.0,1.0,1.25));

   	float fullneck;
   	if(bbn < 0.015)
   	{
   		//neck cylinder, placed at top of box body, scaled so that it goes inside body box for animation purposes
    	mat4 neckMat = mat4(1.0); neckMat[3][1] = 2.0 + animHeight;
    	float neck = cylinderSDF(point, neckMat, vec2(0.5, 2.0));

    	//neck attachment torus, will be intersected with box to create interesting shape
    	mat4 ntMat = mat4(1.0); ntMat[3][1] = 2.0;
    	float nt = torusSDF(point, ntMat, vec2(0.75, 0.5));

    	//neck attachment box, will be intersected with torus to create interesting shape
    	mat4 nbMat = mat4(1.0); nbMat[3][1] = 2.0;
    	float nb = boxSDF(point, ntMat, vec3(0.8, 0.5, 0.8));

    	fullneck = un(neck, intersection(nt, nb));
   	}
   	else
   	{
   		fullneck = bbn;
   	}

   

   	//bounding volume for head
   	mat4 bbhMat = mat4(1.0); bbhMat[3][1] = 4.0 + animHeight;
    float bbh = boxSDF(point, bbhMat, vec3(1.0,1.0,1.5));

   	float fullhead;
   	if(bbh < 0.015)
   	{
   		//head box, placed at top of neck cylinder, moves up and down in animation
    	mat4 headMat = mat4(1.0); headMat[3][1] = 4.0 + animHeight;
    	float head = boxSDF(point, headMat, vec3(1.0));

    	//outer mouth box
    	mat4 omMat = mat4(1.0); omMat[3][1] = 3.5 + animHeight; omMat[3][2] = 1.0;
    	float oMouth = boxSDF(point, omMat, vec3(0.75, 0.2, 0.5));

    	//inner mouth box, to be subtracted
    	mat4 imMat = mat4(1.0); imMat[3][1] = 3.5 + animHeight; imMat[3][2] = 1.2;
    	float iMouth = boxSDF(point, imMat, vec3(0.5, 0.1, 0.5));

    	//outer left eye
    	mat4 oe1Mat = mat4(1.0); oe1Mat[3][0] = -0.5; oe1Mat[3][1] = 4.5 + animHeight; oe1Mat[3][2] = 1.0;
    	float oe1 = sphereSDF(point, oe1Mat, 0.4);

    	//inner left eye
    	mat4 ie1Mat = mat4(1.0); ie1Mat[3][0] = -0.5; ie1Mat[3][1] = 4.5 + animHeight; ie1Mat[3][2] = 1.25;
    		ie1Mat[1][1] = 0.0; ie1Mat[1][2] = 1.0; ie1Mat[2][1] = -1.0; ie1Mat[2][2] = 0.0; //rotating by 90 degrees about x axis
    	float ie1 = cylinderSDF(point, ie1Mat, vec2(0.2, 0.4));

    	//outer right eye
    	mat4 oe2Mat = mat4(1.0); oe2Mat[3][0] = 0.5; oe2Mat[3][1] = 4.5 + animHeight; oe2Mat[3][2] = 1.0;
    	float oe2 = sphereSDF(point, oe2Mat, 0.4);

    	//inner right eye
    	mat4 ie2Mat = mat4(1.0); ie2Mat[3][0] = 0.5; ie2Mat[3][1] = 4.5 + animHeight; ie2Mat[3][2] = 1.25;
    		ie2Mat[1][1] = 0.0; ie2Mat[1][2] = 1.0; ie2Mat[2][1] = -1.0; ie2Mat[2][2] = 0.0; //rotating by 90 degrees about x axis
    	float ie2 = cylinderSDF(point, ie2Mat, vec2(0.2, 0.4));

    	fullhead = un(head, un(subtraction(iMouth, oMouth), un(subtraction(ie1, oe1), subtraction(ie2, oe2))));
   	}
    else
    {
    	fullhead = bbh;
    }



    //bounding volume for antenna
   	mat4 bbaMat = mat4(1.0); bbaMat[3][1] = 5.5 + animHeight;
    float bba = boxSDF(point, bbaMat, vec3(0.2 + 0.2*animHeight,0.8 + 0.2*animHeight,0.2 + 0.2*animHeight));

    float antenna;
    if(bba < 0.015)
    {
    	//antenna cone piece, will be intersected with box for interesting shape
    	mat4 acMat = mat4(1.0); acMat[3][1] = 5.5 + animHeight;
    	float ac = coneSDF(point, acMat, vec3(0.5, 0.5, 0.5));

    	//antenna box piece, will be intersected with cone for interesting shape
    	mat4 abMat = mat4(1.0); abMat[3][1] = 5.5 + animHeight;
    	float ab = boxSDF(point, abMat, vec3(0.15, 0.5, 0.15));

    	//antenna sphere, will animate in scale
    	mat4 asMat = mat4(1.0); asMat[3][1] = 5.75 + animHeight;
    	float as = sphereSDF(point, asMat, 0.2 + 0.2*animHeight);

    	antenna = un(as, intersection(ac, ab));
    }
    else
    {
    	antenna = bba;
    }



    //bounding volume for left arm
   	mat4 bba1Mat = mat4(1.0); bba1Mat[3][0] = 4.0 - 2.0*animHeight; bba1Mat[3][1] = 0.75;
    float bba1 = boxSDF(point, bba1Mat, vec3(3.0, 1.0, 1.0));

    float leftArm;
    if(bba1 < 0.015)
    {
    	//left arm cylinder, moves in and out in animation
    	mat4 arm1Mat = mat4(1.0); arm1Mat[3][0] = 4.0 - 2.0*animHeight; arm1Mat[3][1] = 0.75;
    		arm1Mat[0][0] = 0.0; arm1Mat[0][1] = 1.0; arm1Mat[1][0] = -1.0; arm1Mat[1][1] = 0.0; //rotating by 90 degrees about z axis
    	float arm1 = cylinderSDF(point, arm1Mat, vec2(0.5, 2.0));

    	//hand sphere
    	mat4 hand1Mat = mat4(1.0); hand1Mat[3][0] = 6.25 - 2.0*animHeight; hand1Mat[3][1] = 0.75;
    	float hand1 = sphereSDF(point, hand1Mat, 0.75);

    	//inner hand box, to be subtracted in order to create claw
    	mat4 ihand1Mat = mat4(1.0); ihand1Mat[3][0] = 6.75 - 2.0*animHeight; ihand1Mat[3][1] = 0.75;
    	float ihand1 = boxSDF(point, ihand1Mat, vec3(0.5, 0.3, 1.0));

    	leftArm = un(subtraction(ihand1, hand1), arm1);
    }
    else
    {
    	leftArm = bba1;
    }



    //bounding volume for left arm
   	mat4 bba2Mat = mat4(1.0); bba2Mat[3][0] = -2.0 - 2.0*animHeight; bba2Mat[3][1] = 0.75;
    float bba2 = boxSDF(point, bba2Mat, vec3(3.0, 1.0, 1.0));

    float rightArm;
    if(bba2 < 0.015)
    {
    	//left arm cylinder, moves in and out in animation
    	mat4 arm2Mat = mat4(1.0); arm2Mat[3][0] = -2.0 - 2.0*animHeight; arm2Mat[3][1] = 0.75;
    		arm2Mat[0][0] = 0.0; arm2Mat[0][1] = 1.0; arm2Mat[1][0] = -1.0; arm2Mat[1][1] = 0.0; //rotating by 90 degrees about z axis
    	float arm2 = cylinderSDF(point, arm2Mat, vec2(0.5, 2.0));

    	//hand sphere
    	mat4 hand2Mat = mat4(1.0); hand2Mat[3][0] = -4.25 - 2.0*animHeight; hand2Mat[3][1] = 0.75;
    	float hand2 = sphereSDF(point, hand2Mat, 0.75);

    	//inner hand box, to be subtracted in order to create claw
    	mat4 ihand2Mat = mat4(1.0); ihand2Mat[3][0] = -4.75 - 2.0*animHeight; ihand2Mat[3][1] = 0.75;
    	float ihand2 = boxSDF(point, ihand2Mat, vec3(0.5, 0.3, 1.0));

    	rightArm = un(subtraction(ihand2, hand2), arm2);
    }
    else
    {
    	rightArm = bba2;
    }



    //bounding volume for left leg
   	mat4 bbl1Mat = mat4(1.0); bbl1Mat[3][0] = 1.0; bbl1Mat[3][1] = -2.0 - 2.0*animHeight;
    float bbl1 = boxSDF(point, bbl1Mat, vec3(1.0, 2.0, 1.0));

    float leftLeg;
    if(bbl1 < 0.015)
    {
    	//left leg cylinder, placed at bottom of body box, moves up and down in animation
    	mat4 leg1Mat = mat4(1.0); leg1Mat[3][0] = 1.0; leg1Mat[3][1] = -2.0 - 2.0*animHeight;
    	float leg1 = cylinderSDF(point, leg1Mat, vec2(0.5, 2.0));

    	//left leg top torus, placed a third of the way down left leg, subtracted to create indentation
    	mat4 ll1Mat = mat4(1.0); ll1Mat[3][0] = 1.0; ll1Mat[3][1] = -1.0 - 2.0*animHeight;
    	float ll1 = torusSDF(point, ll1Mat, vec2(0.75, 0.5));

    	//left leg top torus, placed a third of the way down left leg, subtracted to create indentation
    	mat4 ll2Mat = mat4(1.0); ll2Mat[3][0] = 1.0; ll2Mat[3][1] = -3.0 - 2.0*animHeight;
    	float ll2 = torusSDF(point, ll2Mat, vec2(0.75, 0.5));

    	//left foot cone, placed under leg such that point is inside the cylinder and hidden
   	 	mat4 foot1Mat = mat4(1.0); foot1Mat[3][0] = 1.0; foot1Mat[3][1] = -3.0 - 2.0*animHeight;
    	float foot1 = coneSDF(point, foot1Mat, vec3(1.0, 1.0, 1.0));

    	leftLeg = un(foot1, subtraction(ll1, subtraction(ll2, leg1)));
    }
	else
	{
		leftLeg = bbl1;
	}    



	//bounding volume for right leg
   	mat4 bbl2Mat = mat4(1.0); bbl2Mat[3][0] = -1.0; bbl2Mat[3][1] = -4.0 + 2.0*animHeight;
    float bbl2 = boxSDF(point, bbl2Mat, vec3(1.0, 2.0, 1.0));

    float rightLeg;
    if(bbl2 < 0.015)
    {
    	//right leg cylinder, placed at bottom of body box, moves up and down in animation
    	mat4 leg2Mat = mat4(1.0); leg2Mat[3][0] = -1.0; leg2Mat[3][1] = -4.0 + 2.0*animHeight;
    	float leg2 = cylinderSDF(point, leg2Mat, vec2(0.5, 2.0));

    	//left leg top torus, placed a third of the way down left leg, subtracted to create indentation
    	mat4 rl1Mat = mat4(1.0); rl1Mat[3][0] = -1.0; rl1Mat[3][1] = -3.0 + 2.0*animHeight;
    	float rl1 = torusSDF(point, rl1Mat, vec2(0.75, 0.5));

    	//left leg top torus, placed a third of the way down left leg, subtracted to create indentation
    	mat4 rl2Mat = mat4(1.0); rl2Mat[3][0] = -1.0; rl2Mat[3][1] = -5.0 + 2.0*animHeight;
    	float rl2 = torusSDF(point, rl2Mat, vec2(0.75, 0.5));

    	//right foot cone, placed under leg such that point is inside the cylinder and hidden
    	mat4 foot2Mat = mat4(1.0); foot2Mat[3][0] = -1.0; foot2Mat[3][1] = -5.0 + 2.0*animHeight;
    	float foot2 = coneSDF(point, foot2Mat, vec3(1.0, 1.0, 1.0));

    	rightLeg = un(foot2, subtraction(rl1, subtraction(rl2, leg2)));
    }
    else
    {
    	rightLeg = bbl2;
    }

    return un(fullbody, un(fullneck, un(fullhead, un(antenna, un(leftArm, un(rightArm, un(leftLeg, rightLeg)))))));
}

vec3 getNormal(vec3 point)
{
	float epsilon = 0.1;
	float x = sphereTrace(vec3(point.x+epsilon, point.y, point.z)) - sphereTrace(vec3(point.x-epsilon, point.y, point.z));
	float y = sphereTrace(vec3(point.x, point.y+epsilon, point.z)) - sphereTrace(vec3(point.x, point.y-epsilon, point.z));
	float z = sphereTrace(vec3(point.x, point.y, point.z+epsilon)) - sphereTrace(vec3(point.x, point.y, point.z-epsilon));
	return normalize(vec3(x, y, z));
}

vec3 lambert(vec3 norm)
{
	vec3 light = -1.0 * normalize(vec3(2.0, -3.0, -6.0));
	return clamp(dot(light, norm), 0.0, 1.0)*vec3(0.5, 0.5, 0.7);
}

vec4 traceRay()
{
	vec3 ray = createRay();
	vec3 currPt = cameraPos;
    float t = 0.0;
    int count = 0;
    for(int i = 0; i < MAX_ITER; i++)
    {
    	currPt = cameraPos + ray * t;
    	float offset = sphereTrace(currPt);
    	
    	if (abs(offset) < 0.01) {
    		break;
    	}
    	t += offset;
    	++count;
    }

    //return vec4(vec3(float(count)/float(MAX_ITER)), 1.0);
    return vec4(lambert(getNormal(currPt)), 1.0);
}

void main() {

	vec4 color = traceRay();

    gl_FragColor = color;
}