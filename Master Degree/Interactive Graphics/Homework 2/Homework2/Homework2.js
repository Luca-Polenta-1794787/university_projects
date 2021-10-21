"use strict";

var canvas;
var gl;
var program;

var projectionMatrix, projectionMatrixLoc;
var modelViewMatrix;
var nMatrix, nMatrixLoc;

/* MODEL VIEW */
const at = vec3(0.0, 0.0, 0.0);
const up = vec3(0.0, 1.0, 0.0);
var radius = 0.05;
var thetaEx6 = 46;
var phi = -37;
var viewerPos;

var leftOrth = -20;
var rightOrth = 20;
var bottomOrth = -20;
var topOrth = 20;
var nearOrth = -20;
var farOrth = 20;
////////////////////

var instanceMatrix;

var modelViewMatrixLoc;

var vertices = [
    vec4(-0.5, -0.5, 0.5, 1.0),
    vec4(-0.5, 0.5, 0.5, 1.0),
    vec4(0.5,  0.5, 0.5, 1.0),
    vec4(0.5, -0.5, 0.5, 1.0),
    vec4(-0.5, -0.5, -0.5, 1.0),
    vec4(-0.5, 0.5, -0.5, 1.0),
    vec4(0.5, 0.5, -0.5, 1.0),
    vec4(0.5, -0.5, -0.5, 1.0)
];

var coordTorso = [-9.0, -3.95, 0];

/* LIGHT */ 
var lightPosition1 = vec4(-90.0, 30.0, -90.0, 1.0);
var lightPosition2 = vec4(90.0, 30.0, 90.0, 1.0);
var lightAmbient = vec4(0.8, 0.8, 0.8, 1.0);
var lightDiffuse = vec4(0.99, 0.99, 0.99, 1.0);
var lightSpecular = vec4(0.5, 0.5, 0.5, 1.0);

var materialDiffuseSheep = vec4(0.99, 0.99, 0.99, 1.0);

var materialShininess = 100.0;
var materialDiffuseSheepLimbs = vec4(0.2, 0.2, 0.2, 1.0);
var materialAmbientSheepLimbs = vec4(0.8, 0.8, 0.8, 1.0);
var materialSpecularSheepLimbs = vec4(0.1, 0.1, 0.1, 1.0);

var materialDiffuseErba = vec4(0.2, 0.41, 0.18, 1.0);
var materialAmbientErba = vec4(0.3, 0.3, 0.3, 1.0);
var materialSpecularErba = vec4(0.7, 0.7, 0.7, 1.0);

var materialDiffuseLegno = vec4(0.65, 0.38, 0.04, 1.0);
var materialAmbientLegno = vec4(0.3, 0.3, 0.3, 1.0);
var materialSpecularLegno = vec4(0.6, 0.6, 0.6, 1.0);

var materialDiffuseFace = vec4(0.7, 0.7, 0.7, 1.0);
var materialAmbientFace = vec4(0.2, 0.2, 0.2, 1.0);
var materialSpecularFace = vec4(0.2, 0.2, 0.2, 1.0);
////////////////////

/* TORSO AND HEAD */
var torsoId = 0;
var headId  = 1;
var head1Id = 1;
var head2Id = 10;

var torsoHeight = 2.8;
var torsoWidth = 6.0;

var headHeight = 1.6;
var headWidth = 1.3;
////////////////////

/* LEGS */
var leftUpperArmId = 2;
var leftLowerArmId = 3;
var rightUpperArmId = 4;
var rightLowerArmId = 5;
var leftUpperLegId = 6;
var leftLowerLegId = 7;
var rightUpperLegId = 8;
var rightLowerLegId = 9;

var upperArmHeight = 2.2;
var upperArmWidth  = 1.3;

var lowerArmHeight = 1.0;
var lowerArmWidth  = 0.8;

var upperLegHeight = 2.2;
var upperLegWidth  = 1.3;

var lowerLegHeight = 1.0;
var lowerLegWidth  = 0.8;
////////////////////

/* TAIL */
var tailId = 11;
var tailHeight = 0.7;
var tailWidth = 0.5;
////////////////////

/* GROUND */
var groundId = 12;
var groundWidth = 20.0;
var groundHeight = 15.0;
////////////////////

/* FENCE */
var fenceId = 13;
var fenceLeftPoleId = 14;
var fenceRightPoleId = 15;
var fenceMiddleId = 16;
var fencePoleWidth = 0.5;
var fencePoleHeight = 1.8;
////////////////////

/* FACE */
var faceId = 17;
var faceWidth= 1.5;
var faceHeight= 0.95;
////////////////////

/* TEXTURE */
var texCoordsArray = [];
var textureErba;
var textureSheep;
var textureLegno;
var textureFace;

// TEXTURE COORDINATES
var texCoord = [
    vec2(0, 0),
    vec2(0, 1),
    vec2(1, 1),
    vec2(1, 0)
];

/* CONFIGURE TEXTURE */
function configureTexture(image1, image2, image3, image4) {
    textureErba = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, textureErba);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB, gl.RGB, gl.UNSIGNED_BYTE, image1);
    gl.generateMipmap(gl.TEXTURE_2D);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST_MIPMAP_LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
    
    gl.uniform1i(gl.getUniformLocation(program, "uTextureMapErba"), 0);

    textureSheep = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, textureSheep);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB, texSize, texSize, 0, gl.RGB, gl.UNSIGNED_BYTE, image2);
    gl.generateMipmap(gl.TEXTURE_2D);

    gl.uniform1i(gl.getUniformLocation(program, "uTextureMapSheep"), 1);

    textureLegno = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, textureLegno);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB, gl.RGB, gl.UNSIGNED_BYTE, image3);
    gl.generateMipmap(gl.TEXTURE_2D);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST_MIPMAP_LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
    
    gl.uniform1i(gl.getUniformLocation(program, "uTextureMapLegno"), 2);

    textureFace = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, textureFace);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB, gl.RGB, gl.UNSIGNED_BYTE, image4);
    gl.generateMipmap(gl.TEXTURE_2D);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST_MIPMAP_LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
    
    gl.uniform1i(gl.getUniformLocation(program, "uTextureMapFace"), 3);
}
////////////////////

/* BUMP TEXTURE */
var texSize = 256;
var tangentsArray = [];
var data = new Array()
var howMuch = 10.0;
var randomNumber;
for (var i = 0; i<= texSize; i++)  data[i] = new Array();
for (var i = 0; i<=texSize; i++){
    for (var j=0; j<=texSize; j++){
        randomNumber = Math.random();
        if(randomNumber<0.5){
            data[i][j] = 0.0;
        }else{
            data[i][j] = howMuch;
        }
    }
}
    
// Bump Map Normals
var normalst = new Array()
      for (var i=0; i<texSize; i++)  normalst[i] = new Array();
      for (var i=0; i<texSize; i++) for ( var j = 0; j < texSize; j++)
         normalst[i][j] = new Array();
      for (var i=0; i<texSize; i++) for ( var j = 0; j < texSize; j++) {
         normalst[i][j][0] = data[i][j]-data[i+1][j];
         normalst[i][j][1] = data[i][j]-data[i][j+1];
         normalst[i][j][2] = 1;
      }

// Scale to Texture Coordinates
      for (var i=0; i<texSize; i++) for (var j=0; j<texSize; j++) {
         var d = 0;
         for(k=0;k<3;k++) d+=normalst[i][j][k]*normalst[i][j][k];
         d = Math.sqrt(d);
         for(k=0;k<3;k++) normalst[i][j][k]= 0.5*normalst[i][j][k]/d + 0.5;
      }
   
// Normal Texture Array
var normalsBumpTexture = new Uint8Array(3*texSize*texSize);
      for ( var i = 0; i < texSize; i++ )
         for ( var j = 0; j < texSize; j++ )
            for(var k =0; k<3; k++)
               normalsBumpTexture[3*texSize*i+3*j+k] = 255*normalst[i][j][k];

////////////////////


var numNodes = 19;
var numAngles = 11;
var angle = 0;

var theta = [0, 0, -170, -5, 160, 15, 180, 15, 155, -15, 15, 210, 0, 90, 0, 0, 90, 180]

var numVertices = 24;
var stack = [];
var figure = [];

for( var i=0; i<numNodes; i++) figure[i] = createNode(null, null, null, null);

var vBuffer;
var modelViewLoc;

var pointsArray = [];
var normalsArray = [];

//-------------------------------------------
function scale4(a, b, c) {
   var result = mat4();
   result[0] = a;
   result[5] = b;
   result[10] = c;
   return result;
}
//--------------------------------------------

function createNode(transform, render, sibling, child){
    var node = {
        transform: transform,
        render: render,
        sibling: sibling,
        child: child,
    }
    return node;
}

function initNodes(Id) {

    var m = mat4();

    switch(Id) {
        case torsoId:
            m = translate(coordTorso[0], coordTorso[1], coordTorso[2]);
            // The following 3 lines reduce the size of the sheep
            m[0][0]=0.8;
            m[1][1]=0.8;
            m[2][2]=0.8;
            m = mult(m, rotate(theta[torsoId], vec3(0, 0, 1)));
            figure[torsoId] = createNode( m, torso, groundId, headId);
        break;

        case headId:
        case head1Id:
        case head2Id:
            m = translate(3.5, -0.5*headHeight+torsoHeight, 0.0);  
            m = mult(m, rotate(theta[head1Id], vec3(0, 0, 1)));
            m = mult(m, rotate(theta[head1Id], vec3(0, 1, 0)));
            m = mult(m, rotate(theta[head2Id], vec3(0, 0, 1)));
            m = mult(m, translate(0.0, -0.5*headHeight, 0.0));
            figure[headId] = createNode( m, head, leftUpperArmId, null);
        break;

        case faceId:
            m = translate(-1.1, faceHeight, 0.0);
            m = mult(m, rotate(theta[faceId], vec3(0, 0, 1)));
            figure[faceId] = createNode( m, face, null, null);
            break; 

        case leftUpperArmId:
            m = translate((0.8*torsoHeight/3)+0.2, 0.13*torsoWidth, 2.0);
            m = mult(m, rotate(theta[leftUpperArmId], vec3(0, 0, 1)));
            figure[leftUpperArmId] = createNode( m, leftUpperArm, rightUpperArmId, leftLowerArmId );
        break;

        case rightUpperArmId:
            m = translate(0.8*torsoHeight/2, 0.13*torsoWidth, -2.0);
            m = mult(m, rotate(theta[rightUpperArmId], vec3(0, 0, 1)));
            figure[rightUpperArmId] = createNode( m, rightUpperArm, leftUpperLegId, rightLowerArmId );
        break;

        case leftUpperLegId:
            m = translate(-(0.3*torsoHeight/3+upperLegWidth), 0.33*upperLegHeight, -2.0);
            m = mult(m, rotate(theta[leftUpperLegId], vec3(0, 0, 1)));
            figure[leftUpperLegId] = createNode( m, leftUpperLeg, rightUpperLegId, leftLowerLegId );
        break;

        case rightUpperLegId:
            m = translate(-(0.9*torsoHeight/3+0.1*upperLegWidth)-0.5, 0.33*upperLegHeight, 2.0);
            m = mult(m, rotate(theta[rightUpperLegId], vec3(0, 0, 1)));
            figure[rightUpperLegId] = createNode( m, rightUpperLeg, tailId, rightLowerLegId );
        break;

        case leftLowerArmId:
            m = translate(0.0, upperArmHeight, 0.0);
            m = mult(m, rotate(theta[leftLowerArmId], vec3(0, 0, 1)));
            figure[leftLowerArmId] = createNode( m, leftLowerArm, null, null );
        break;

        case rightLowerArmId:
            m = translate(0.0, upperArmHeight, 0.0);
            m = mult(m, rotate(theta[rightLowerArmId], vec3(0, 0, 1)));
            figure[rightLowerArmId] = createNode( m, rightLowerArm, null, null );
        break;

        case leftLowerLegId:
            m = translate(0.0, upperLegHeight, 0.0);
            m = mult(m, rotate(theta[leftLowerLegId],vec3(0, 0, 1)));
            figure[leftLowerLegId] = createNode( m, leftLowerLeg, null, null );
        break;

        case rightLowerLegId:
            m = translate(0.0, upperLegHeight, 0.0);
            m = mult(m, rotate(theta[rightLowerLegId], vec3(0, 0, 1)));
            figure[rightLowerLegId] = createNode(m, rightLowerLeg, null, null );
        break;

        case tailId:
            m = translate(-3.4, tailHeight+0.4*torsoHeight, 0.0);
            m = mult(m, rotate(theta[tailId], vec3(0, 0, 1)));
            m = mult(m, rotate(180, vec3(0, 1, 0)));
            figure[tailId] = createNode( m, tail, null, null );
        break;

        /* GROUND */
        case groundId:
            m = translate(0, -0.91*groundHeight, 0.0);
            m = mult(m, rotate(theta[groundId], vec3(0, 0, 1)));
            figure[groundId] = createNode(m, ground, fenceId, null );
        break;
        ////////////////////

        /* FENCE */
        case fenceId:
            m = translate(0.6, -2.0*fencePoleHeight, 0.0);
            m = mult(m, rotate(theta[fenceId], vec3(0, 0, 1)));
            m = mult(m, rotate(theta[fenceId], vec3(1, 0, 0)));
            figure[fenceId] = createNode(m, fence, fenceLeftPoleId, null);
        break;

        case fenceLeftPoleId:
            m = translate(0.6, -1.0*fencePoleHeight, 0.0);
            m = mult(m, rotate(theta[fenceLeftPoleId], vec3(0, 0, 1)));
            figure[fenceLeftPoleId] = createNode(m, fenceLeftPole, fenceRightPoleId, null);
        break;

        case fenceRightPoleId:
            m = translate(0.6, 1.0*fencePoleHeight, 0.0);
            m = mult(m, rotate(theta[fenceRightPoleId], vec3(0, 0, 1)));
            figure[fenceRightPoleId] = createNode(m, fenceRightPole, fenceMiddleId, null );
        break;

        case fenceMiddleId:
            m = translate(0.6, -1.0*fencePoleHeight, 0.0);
            m = mult(m, rotate(theta[fenceId], vec3(0, 0, 1)));
            m = mult(m, rotate(theta[fenceId], vec3(1, 0, 0)));
            figure[fenceMiddleId] = createNode(m, fenceMiddle, null, null );
        break;
        ////////////////////
    }
}

function setUniformTexture(flagTexErba, flagTexBump, flagTexLegno, flagFace){
    gl.uniform1f(gl.getUniformLocation(program, "uTextureFace"), false);
    gl.uniform4fv(gl.getUniformLocation(program, "uLightPosition1"), lightPosition1);
    gl.uniform4fv(gl.getUniformLocation(program, "uLightPosition2"), lightPosition2);
    gl.uniform1f(gl.getUniformLocation(program, "uTextureErba"), flagTexErba);
    gl.uniform1f(gl.getUniformLocation(program, "uTextureBump"), flagTexBump);
    gl.uniform1f(gl.getUniformLocation(program, "uTextureLegno"), flagTexLegno);
    if(flagTexErba){
        var diffuseProductErba = mult(lightDiffuse, materialDiffuseErba);
        var ambientProductErba = mult(lightAmbient, materialAmbientErba);
        var specularProductErba = mult(lightSpecular, materialSpecularErba);
        gl.uniform4fv(gl.getUniformLocation(program, "uAmbientProduct"), diffuseProductErba);
        gl.uniform4fv(gl.getUniformLocation(program, "uDiffuseProduct"), ambientProductErba);
        gl.uniform4fv(gl.getUniformLocation(program, "uSpecularProduct"), specularProductErba);
        gl.uniform1f(gl.getUniformLocation(program, "uShininess"), materialShininess);
    }else if(flagTexBump){
        var diffuseProductSheep = mult(lightDiffuse, materialDiffuseSheep);
        gl.uniform4fv(gl.getUniformLocation(program, "uDiffuseProduct"), diffuseProductSheep);
    }else if(flagTexLegno){
        var diffuseProductLegno = mult(lightDiffuse, materialDiffuseLegno);
        var ambientProductLegno = mult(lightAmbient, materialAmbientLegno);
        var specularProductLegno = mult(lightSpecular, materialSpecularLegno);
        gl.uniform4fv(gl.getUniformLocation(program, "uAmbientProduct"), diffuseProductLegno);
        gl.uniform4fv(gl.getUniformLocation(program, "uDiffuseProduct"), ambientProductLegno);
        gl.uniform4fv(gl.getUniformLocation(program, "uSpecularProduct"), specularProductLegno);
        gl.uniform1f(gl.getUniformLocation(program, "uShininess"), materialShininess);
    }else if(flagFace){
        gl.uniform1f(gl.getUniformLocation(program, "uTextureFace"), true);
        var diffuseProductFace = mult(lightDiffuse, materialDiffuseFace);
        var ambientProductFace = mult(lightAmbient, materialAmbientFace);
        var specularProductFace = mult(lightSpecular, materialSpecularFace);
        gl.uniform4fv(gl.getUniformLocation(program, "uAmbientProduct"), diffuseProductFace);
        gl.uniform4fv(gl.getUniformLocation(program, "uDiffuseProduct"), ambientProductFace);
        gl.uniform4fv(gl.getUniformLocation(program, "uSpecularProduct"), specularProductFace);
        gl.uniform1f(gl.getUniformLocation(program, "uShininess"), materialShininess);
    }else{
        var diffuseProductSheepLimbs = mult(lightDiffuse, materialDiffuseSheepLimbs);
        var ambientProductSheepLimbs = mult(lightAmbient, materialAmbientSheepLimbs);
        var specularProductSheepLimbs = mult(lightSpecular, materialSpecularSheepLimbs);
        gl.uniform4fv(gl.getUniformLocation(program, "uAmbientProduct"), diffuseProductSheepLimbs);
        gl.uniform4fv(gl.getUniformLocation(program, "uDiffuseProduct"), ambientProductSheepLimbs);
        gl.uniform4fv(gl.getUniformLocation(program, "uSpecularProduct"), specularProductSheepLimbs);
        gl.uniform1f(gl.getUniformLocation(program, "uShininess"), materialShininess);
    }
}

function traverse(Id) {
   if(Id == null) return;
   stack.push(modelViewMatrix);
   modelViewMatrix = mult(modelViewMatrix, figure[Id].transform);
   figure[Id].render();
   if(figure[Id].child != null) traverse(figure[Id].child);
    modelViewMatrix = stack.pop();
   if(figure[Id].sibling != null) traverse(figure[Id].sibling);
}

function torso() {
    setUniformTexture(false, true, false, false);
    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.3*torsoWidth, 0.0));
    instanceMatrix = mult(instanceMatrix, scale( torsoWidth, torsoHeight, torsoWidth-0.7));
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function head() {
    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5*headHeight, 0.0 ));
	instanceMatrix = mult(instanceMatrix, scale(headWidth, headHeight, headWidth));
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
    for(var i =0; i<6; i++){
        if(i==1){
            setUniformTexture(false, false, false, true);
        }else{
            setUniformTexture(false, true, false, false);
        }
        gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
    }
}

function face() {
    setUniformTexture(false, false, false, false);
    instanceMatrix = mult(modelViewMatrix, translate(-0.7, 0.45*faceHeight, 0.0 ));
	instanceMatrix = mult(instanceMatrix, scale(faceWidth, faceHeight, faceWidth) );
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function leftUpperArm() {
    setUniformTexture(false, true, false, false);
    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5 * upperArmHeight, 0.0) );
	instanceMatrix = mult(instanceMatrix, scale(upperArmWidth, upperArmHeight, upperArmWidth-0.1) );
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function leftLowerArm() {
    setUniformTexture(false, false, false, false);
    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5 * lowerArmHeight, 0.0) );
	instanceMatrix = mult(instanceMatrix, scale(lowerArmWidth, lowerArmHeight, lowerArmWidth) );
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function rightUpperArm() {
    setUniformTexture(false, true, false, false);
    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5 * upperArmHeight, 0.0) );
	instanceMatrix = mult(instanceMatrix, scale(upperArmWidth, upperArmHeight, upperArmWidth-0.1) );
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function rightLowerArm() {
    setUniformTexture(false, false, false, false);
    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5 * lowerArmHeight, 0.0) );
	instanceMatrix = mult(instanceMatrix, scale(lowerArmWidth, lowerArmHeight, lowerArmWidth) );
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function leftUpperLeg() {
    setUniformTexture(false, true, false, false);
    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5 * upperLegHeight, 0.0) );
	instanceMatrix = mult(instanceMatrix, scale(upperLegWidth, upperLegHeight, upperLegWidth-0.1) );
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function leftLowerLeg() {
    setUniformTexture(false, false, false, false);
    instanceMatrix = mult(modelViewMatrix, translate( 0.0, 0.5 * lowerLegHeight, 0.0) );
	instanceMatrix = mult(instanceMatrix, scale(lowerLegWidth, lowerLegHeight, lowerLegWidth) );
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function rightUpperLeg() {
    setUniformTexture(false, true, false, false);
    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5 * upperLegHeight, 0.0) );
	instanceMatrix = mult(instanceMatrix, scale(upperLegWidth, upperLegHeight, upperLegWidth-0.1) );
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function rightLowerLeg() {
    setUniformTexture(false, false, false, false);
    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5 * lowerLegHeight, 0.0) );
	instanceMatrix = mult(instanceMatrix, scale(lowerLegWidth, lowerLegHeight, lowerLegWidth) )
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function tail() {
    setUniformTexture(false, true, false, false);
    instanceMatrix = mult(modelViewMatrix, translate(0.2, -0.15*tailHeight, 0.0) );
	instanceMatrix = mult(instanceMatrix, scale(tailWidth, 2.3*tailHeight, tailWidth) )
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function ground() {
    setUniformTexture(true, false, false, false);
    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5 * groundHeight, 0.0) );
	instanceMatrix = mult(instanceMatrix, scale(1.5*groundWidth, 0.05*groundHeight, groundWidth) )
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function fence() {
    setUniformTexture(false, false, true, false);
    instanceMatrix = mult(modelViewMatrix, translate(1.2, 0.01*fencePoleHeight, 0.49) );
	instanceMatrix = mult(instanceMatrix, scale(fencePoleWidth, 3*fencePoleHeight, fencePoleWidth) )
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function fenceLeftPole() {
    setUniformTexture(false, false, true, false);
    instanceMatrix = mult(modelViewMatrix, translate(0.5, -1.3*fencePoleHeight, -2.2) );
	instanceMatrix = mult(instanceMatrix, scale(fencePoleWidth, 2*fencePoleHeight, fencePoleWidth) )
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function fenceRightPole() {
    setUniformTexture(false, false, true, false);
    instanceMatrix = mult(modelViewMatrix, translate(0.5, -3.3*fencePoleHeight, 2.2) );
	instanceMatrix = mult(instanceMatrix, scale(fencePoleWidth, 2*fencePoleHeight, fencePoleWidth) )
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function fenceMiddle() {
    setUniformTexture(false, false, true, false);
    instanceMatrix = mult(modelViewMatrix, translate(1.5, 0.01*fencePoleHeight, 0.49) );
	instanceMatrix = mult(instanceMatrix, scale(fencePoleWidth, 3*fencePoleHeight, fencePoleWidth) )
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function quad(a, b, c, d) {
    var t1 = subtract(vertices[b], vertices[a]);
    var t2 = subtract(vertices[c], vertices[b]);
    var normal = cross(t1, t2);
    normal = vec3(normal);

     pointsArray.push(vertices[a]);
     texCoordsArray.push(texCoord[0]);
     normalsArray.push(normal);
     tangentsArray.push(t1);

     pointsArray.push(vertices[b]);
     texCoordsArray.push(texCoord[1]);
     normalsArray.push(normal);
     tangentsArray.push(t1);

     pointsArray.push(vertices[c]);
     texCoordsArray.push(texCoord[2]);
     normalsArray.push(normal);
     tangentsArray.push(t1);

     pointsArray.push(vertices[d]);
     texCoordsArray.push(texCoord[3]);
     normalsArray.push(normal);
     tangentsArray.push(t1);
}


function cube() {
    quad( 1, 0, 3, 2 );
    quad( 2, 3, 7, 6 );
    quad( 3, 0, 4, 7 );
    quad( 6, 5, 1, 2 );
    quad( 4, 5, 6, 7 );
    quad( 5, 4, 0, 1 );
}

var imageErba = new Image();
imageErba.src = "erba.jpg"
var imageLegno = new Image();
imageLegno.src = "legno.png"
var imageFace = new Image();
imageFace.src = "face.jpg"

window.onload = function init() {
    canvas = document.getElementById( "gl-canvas" );

    gl = canvas.getContext('webgl2');
    if (!gl) { alert( "WebGL 2.0 isn't available" ); }

    gl.viewport(0, 0, canvas.width, canvas.height );
    gl.clearColor(0.52, 0.8, 0.92, 1.0); // color of the sky
    gl.enable(gl.DEPTH_TEST);

    //
    //  Load shaders and initialize attribute buffers
    //
    program = initShaders(gl, "vertex-shader", "fragment-shader");

    gl.useProgram(program);

    instanceMatrix = mat4();

    projectionMatrix = ortho(leftOrth, rightOrth, bottomOrth, topOrth, nearOrth, farOrth);
    viewerPos = vec3(radius*Math.sin(thetaEx6)*Math.cos(phi), radius*Math.sin(thetaEx6)*Math.sin(phi), radius*Math.cos(thetaEx6));
 
    modelViewMatrix = lookAt(viewerPos, at , up);

    gl.uniformMatrix4fv(gl.getUniformLocation( program, "modelViewMatrix"), false, flatten(modelViewMatrix)  );
    gl.uniformMatrix4fv( gl.getUniformLocation( program, "projectionMatrix"), false, flatten(projectionMatrix)  );

    modelViewMatrixLoc = gl.getUniformLocation(program, "modelViewMatrix")
    nMatrixLoc = gl.getUniformLocation(program, "uNormalMatrix");
    projectionMatrixLoc = gl.getUniformLocation(program, "projectionMatrix")
    nMatrix = normalMatrix(modelViewMatrix, true);

    gl.uniformMatrix3fv(nMatrixLoc, false, flatten(nMatrix));

    cube();


    /* TEXTURE */
    var tBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, tBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, flatten(texCoordsArray), gl.STATIC_DRAW);

    var texCoordLoc = gl.getAttribLocation(program, "aTexCoord");
    gl.vertexAttribPointer(texCoordLoc, 2, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(texCoordLoc);
    
    configureTexture(imageErba, normalsBumpTexture, imageLegno, imageFace);

    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, textureErba);

    gl.activeTexture(gl.TEXTURE1);
    gl.bindTexture(gl.TEXTURE_2D, textureSheep);

    gl.activeTexture(gl.TEXTURE2);
    gl.bindTexture(gl.TEXTURE_2D, textureLegno);

    gl.activeTexture(gl.TEXTURE3);
    gl.bindTexture(gl.TEXTURE_2D, textureFace);


    vBuffer = gl.createBuffer();
    gl.bindBuffer( gl.ARRAY_BUFFER, vBuffer );
    gl.bufferData(gl.ARRAY_BUFFER, flatten(pointsArray), gl.STATIC_DRAW);

    var positionLoc = gl.getAttribLocation( program, "aPosition" );
    gl.vertexAttribPointer( positionLoc, 4, gl.FLOAT, false, 0, 0 );
    gl.enableVertexAttribArray( positionLoc );

    var nBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, nBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, flatten(normalsArray), gl.STATIC_DRAW);
    
    var normalLoc = gl.getAttribLocation(program, "aNormal");
    gl.vertexAttribPointer(normalLoc, 3, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(normalLoc);
    
    var tangBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, tangBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, flatten(tangentsArray), gl.STATIC_DRAW);

    var tangentLoc = gl.getAttribLocation(program, "aTangent");
    gl.vertexAttribPointer(tangentLoc, 4, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(tangentLoc);
    ////////////////////

    /* HANDLING BUTTONS FOR THE ANIMATION */
    document.getElementById("startAnimationButton").onclick = function(event) {
        setInterval(function() {
            myAnimation();
        }, 32);
        document.getElementById("startAnimationButton").disabled=true;
        document.getElementById("startAnimationSlowerButton").disabled=true;
    };
    document.getElementById("startAnimationSlowerButton").onclick = function(event) {
        setInterval(function() {
            myAnimation();
        }, 175);
        document.getElementById("startAnimationButton").disabled=true;
        document.getElementById("startAnimationSlowerButton").disabled=true;
    };

    document.getElementById("restartAnimationButton").onclick = function(event) {
        resetEnv();
    };
    ////////////////////

    /* HANDLING BUTTONS FOR THE CAMERA */
    document.getElementById("sliderRadius").onchange = function(event){radius = event.target.value; console.log("radius = "+radius); };
    document.getElementById("sliderTheta").onchange = function(event){thetaEx6 = event.target.value; console.log("theta = "+thetaEx6); };
    document.getElementById("sliderPhi").onchange = function(event){phi = event.target.value; console.log("phi = "+phi); };
    document.getElementById("sliderWidth").onchange = function(event){
        leftOrth = -event.target.value/2;
        rightOrth = event.target.value/2;
        console.log("left = "+leftOrth);
        console.log("right = "+rightOrth);
    };
    document.getElementById("sliderHeight").onchange = function(event){
        bottomOrth = -event.target.value/2;
        topOrth = event.target.value/2;
        console.log("bottom = "+bottomOrth);
        console.log("top = "+topOrth);
    };
    document.getElementById("sliderDepth").onchange = function(event){
        nearOrth = -event.target.value/2;;
        farOrth = event.target.value/2;;
        console.log("near = "+nearOrth);
        console.log("far = "+farOrth);
    };
    ////////////////////

    for(i=0; i<numNodes; i++) initNodes(i);

    render();
}

/* FUNCTION FOR THE ANIMATION */
var changeOfAlternatingLegs = true;
var jump = false;
var inclinazione = true;
function myAnimation(){
    if(coordTorso[0]<-3.5 || (coordTorso[0]>=5.25 && coordTorso[0]<10.2)){
        coordTorso[0] +=0.08;
        initNodes(torsoId);
        if (changeOfAlternatingLegs){
            theta[leftUpperArmId] -= 5;
            initNodes(leftUpperArmId);

            theta[rightUpperArmId] += 5; 
            initNodes(rightUpperArmId);

            theta[leftLowerArmId] += 4; 
            initNodes(leftLowerArmId);

            theta[rightLowerArmId] -= 7; 
            initNodes(rightLowerArmId);

            theta[leftUpperLegId] -= 5;
            initNodes(leftUpperLegId);

            theta[rightUpperLegId] += 5; 
            initNodes(rightUpperLegId);

            theta[leftLowerLegId] -= 4; 
            initNodes(leftLowerLegId);

            theta[rightLowerLegId] += 7; 
            initNodes(rightLowerLegId);
            
            if (theta[leftUpperArmId] == -195){
                changeOfAlternatingLegs = false;
            }
        }
        else{
            theta[leftUpperArmId] += 5;
            initNodes(leftUpperArmId);

            theta[rightUpperArmId] -= 5;
            initNodes(rightUpperArmId);

            theta[leftLowerArmId] -= 4;
            initNodes(leftLowerArmId);

            theta[rightLowerArmId] += 7;
            initNodes(rightLowerArmId);

            theta[leftUpperLegId] += 5;
            initNodes(leftUpperLegId);

            theta[rightUpperLegId] -= 5; 
            initNodes(rightUpperLegId);

            theta[leftLowerLegId] += 4; 
            initNodes(leftLowerLegId);

            theta[rightLowerLegId] -= 7; 
            initNodes(rightLowerLegId);
            
            if (theta[leftUpperArmId] == -170){
                changeOfAlternatingLegs = true;
            }
        }
    }else if(theta[torsoId]>=-30 && coordTorso[0]<-2.0){
        theta[torsoId] -= 3;
        coordTorso[1] += 0.04;
        initNodes(torsoId);

        if(inclinazione){
            theta[leftUpperLegId] = 180;
            initNodes(leftUpperLegId);
    
            theta[rightUpperLegId] = 180;
            initNodes(rightUpperLegId);
    
            theta[leftLowerLegId] = 0;
            initNodes(leftLowerLegId);
    
            theta[rightLowerLegId] = 0;
            initNodes(rightLowerLegId);
            inclinazione = false;
        }else{
            theta[leftUpperLegId] += 5;
            initNodes(leftUpperLegId);

            theta[rightUpperLegId] += 5; 
            initNodes(rightUpperLegId);

            theta[leftLowerLegId] += 5; 
            initNodes(leftLowerLegId);

            theta[rightLowerLegId] += 5; 
            initNodes(rightLowerLegId);
        }
    }else if(coordTorso[0]<1.7){
        coordTorso[0] += 0.06;
        coordTorso[1] += 0.07;
        initNodes(torsoId);
        inclinazione = true;
    }else if(coordTorso[1]>-2.95){
        if(theta[torsoId]<30){
            theta[torsoId] += 4;
        }
        coordTorso[0] += 0.1;
        coordTorso[1] -= 0.16;
        initNodes(torsoId);
        if(inclinazione){
            theta[leftUpperLegId] = 180;
            initNodes(leftUpperLegId);

            theta[leftLowerLegId] = 35;
            initNodes(leftLowerLegId);
    
            theta[rightUpperLegId] = 160;
            initNodes(rightUpperLegId);
        
            theta[rightLowerLegId] = -5;
            initNodes(rightLowerLegId);

            theta[leftUpperArmId] = 140;
            initNodes(leftUpperArmId);
    
            theta[rightUpperArmId] = 140;
            initNodes(rightUpperArmId);
    
            theta[leftLowerArmId] = 0;
            initNodes(leftLowerArmId);
    
            theta[rightLowerArmId] = 0;
            initNodes(rightLowerArmId);
            inclinazione = false;
        }
    }else if(theta[torsoId]>0){
        theta[torsoId] -= 5;
        coordTorso[1] -= 0.18;
        initNodes(torsoId);

        if(inclinazione){
            theta[leftUpperLegId] = 180;
            initNodes(leftUpperLegId);
    
            theta[rightUpperLegId] = 180;
            initNodes(rightUpperLegId);
    
            theta[leftLowerLegId] = 0;
            initNodes(leftLowerLegId);
    
            theta[rightLowerLegId] = 0;
            initNodes(rightLowerLegId);
            inclinazione = false;
        }
    }else if(coordTorso[1]<=-2.95 && coordTorso[0]<5.5){
        theta[torsoId] = 0;
        coordTorso[0] += 0.1;
        coordTorso[1] = -3.95;
        initNodes(torsoId);

        theta[leftUpperArmId] = -170;
        initNodes(leftUpperArmId);

        theta[leftLowerArmId] = -5;
        initNodes(leftLowerArmId);

        theta[rightUpperArmId] = 160;
        initNodes(rightUpperArmId);

        theta[rightLowerArmId] = 15;
        initNodes(rightLowerArmId);

        theta[leftUpperLegId] = 180;
        initNodes(leftUpperLegId);

        theta[leftLowerLegId] = 15;
        initNodes(leftLowerLegId);

        theta[rightUpperLegId] = 155;
        initNodes(rightUpperLegId);
    
        theta[rightLowerLegId] = -15;
        initNodes(rightLowerLegId);

        changeOfAlternatingLegs = true;
    }
}

/* RELOAD THE PAGE TO RESET ANIMATION */
function resetEnv(){
    window.location.reload();
}
////////////////////

var render = function() {
        gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
        
        projectionMatrix = ortho(leftOrth, rightOrth, bottomOrth, topOrth, nearOrth, farOrth);
        viewerPos = vec3(radius*Math.sin(thetaEx6)*Math.cos(phi), radius*Math.sin(thetaEx6)*Math.sin(phi), radius*Math.cos(thetaEx6));
        modelViewMatrix = lookAt(viewerPos, at , up);
        nMatrix = normalMatrix(modelViewMatrix, true);
        gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(modelViewMatrix));
        gl.uniformMatrix3fv(nMatrixLoc, false, flatten(nMatrix));
        gl.uniformMatrix4fv(projectionMatrixLoc, false, flatten(projectionMatrix));

        traverse(torsoId);

        requestAnimationFrame(render);
}
