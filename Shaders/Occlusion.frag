uniform sampler2D realRGBTexture;
uniform sampler2D realDepthTexture;
uniform sampler2D virtualRGBTexture;
uniform sampler2D virtualDepthTexture;
uniform sampler2D curvatureMap;
uniform sampler2D contoursMap;
uniform sampler2D backgroundMap;
uniform sampler2D subtractionMap;
uniform int ARPolygonal;
uniform int ARFromKinectFusionVolume;
uniform int ARFromVolumeRendering;
uniform int alphaBlending;
uniform int ghostViewBasedOnCurvatureMap;
uniform int ghostViewBasedOnDistanceFalloff;
uniform int ghostViewBasedOnClipping;
uniform int ghostViewBasedOnSubtractionMaskCase1;
uniform int ghostViewBasedOnSubtractionMaskCase2;
uniform float curvatureWeight;
uniform float distanceFalloffWeight;
uniform float clippingWeight;
uniform float grayLevelWeight;
uniform vec2 focusPoint;
uniform float focusRadius;

vec4 computeFragmentColorARPolygonal(vec4 realDepth, vec4 virtualDepth, vec4 realRGB, vec4 virtualRGB, float alpha, float threshold) {
	
	if(virtualDepth.r == 0.0)
		return realRGB;
	else if((virtualDepth.r - threshold) < realDepth.r)
		return virtualRGB * (1 - alpha) + realRGB * alpha;
	else
		return realRGB;

}

vec4 computeFragmentColorARFromKinectFusionVolume(vec4 realDepth, vec4 virtualDepth, vec4 realRGB, vec4 virtualRGB, float alpha, float threshold) {

	if(virtualDepth.r == 0.0)
		return realRGB;
	else if(virtualRGB.r == 0.0 && virtualRGB.g == 0.0 && virtualRGB.b == 0.0)
		return realRGB;
	else if((virtualDepth.r - threshold) < realDepth.r)
		return virtualRGB * (1 - alpha) + realRGB * alpha;
	else
		return realRGB;

}

vec4 computeFragmentColorARFromVolumeRendering(vec4 realDepth, vec4 virtualDepth, vec4 realRGB, vec4 virtualRGB, float alpha, float threshold) {
	
	if(virtualDepth.r == 0.0)
		return realRGB;
	else if(virtualRGB.r == 0.0 && virtualRGB.g == 0.0 && virtualRGB.b == 0.0)
		return realRGB;
	else if((virtualDepth.r - threshold) < realDepth.r && distance(gl_FragCoord.xy, focusPoint) < focusRadius)
		return virtualRGB * (1 - alpha) + realRGB * alpha;
	else if(realDepth.r == 0.0)
		return virtualRGB * (1 - alpha) + realRGB * alpha;
	else
		return realRGB;

}

vec4 computeFragmentColorARFromVolumeRendering(vec4 realDepth, vec4 virtualDepth, vec4 realRGB, vec4 virtualRGB, float alpha, float threshold, vec4 backgroundRGB, 
	vec4 subtractionRGB, float grayLevelWeight) {
	
	/*
	if(virtualDepth.r == 0.0)
		return realRGB;
	else if(realDepth.r < (virtualDepth.r - threshold) && realDepth.r != 0.0) //holes
		return realRGB;
	else if(subtractionRGB.r == 1.0 && subtractionRGB.g == 1.0 && subtractionRGB.b == 1.0) {
		//if(virtualRGB.r == 0.0 && virtualRGB.g == 0.0 && virtualRGB.b == 0.0)
		float grayLevel = (virtualRGB.r + virtualRGB.g + virtualRGB.b)/3;
		//if(grayLevel > clippingWeight)
			//return backgroundRGB;
		//else	
		alpha = grayLevel;
		if(grayLevel > clippingWeight/10.0)
		return virtualRGB * (1 - alpha) + backgroundRGB * alpha;
		else 
		return virtualRGB;
	} else 
		return realRGB;
	*/

	if(virtualDepth.r == 0.0)
		return realRGB;
	else if(realDepth.r < (virtualDepth.r - threshold) && realDepth.r != 0.0) //holes
		return realRGB;
	else if(subtractionRGB.r == 1.0 && subtractionRGB.g == 1.0 && subtractionRGB.b == 1.0) {
		float grayLevel = (virtualRGB.r + virtualRGB.g + virtualRGB.b)/3;
		alpha = grayLevel;
		if(grayLevel > grayLevelWeight)
			return virtualRGB * (1 - alpha) + backgroundRGB * alpha;
		else 
			return virtualRGB;
	} else 
		return realRGB;

}

vec4 computeFragmentColorARFromVolumeRendering(vec4 realDepth, vec4 virtualDepth, vec4 realRGB, vec4 virtualRGB, float alpha, float threshold, vec4 backgroundRGB, 
	vec4 subtractionRGB) {
	
	if(virtualDepth.r == 0.0)
		return realRGB;
	else if(realDepth.r < (virtualDepth.r - threshold) && realDepth.r != 0.0) //holes
		return realRGB;
	else if(subtractionRGB.r == 1.0 && subtractionRGB.g == 1.0 && subtractionRGB.b == 1.0) {
		if(virtualRGB.r == 0.0 && virtualRGB.g == 0 && virtualRGB.b == 0)
			return backgroundRGB;
		else
			return virtualRGB;
	} else 
		return realRGB;

}

void main (void)  
{

	vec4 realDepth = texture2D(realDepthTexture, vec2(gl_TexCoord[0].s, gl_TexCoord[0].t));
	vec4 virtualDepth = texture2D(virtualDepthTexture, vec2(gl_TexCoord[0].s, gl_TexCoord[0].t));
	
	vec4 realRGB = texture2D(realRGBTexture, gl_TexCoord[0].st);
	vec4 virtualRGB = texture2D(virtualRGBTexture, vec2(gl_TexCoord[0].s, 1 - gl_TexCoord[0].t));
	
	float threshold = 0.01;

	vec4 fragColor = vec4(0, 0, 0, 0);

	float alpha = 0;

	if(alphaBlending == 1) {
		alpha = 0.2;
	} else {
		if(ghostViewBasedOnCurvatureMap == 1)
			alpha = texture2D(curvatureMap, gl_TexCoord[0].st).r * curvatureWeight;
		if(ghostViewBasedOnDistanceFalloff == 1)
			alpha = max(alpha, pow(distance(gl_FragCoord.xy, focusPoint)/focusRadius, distanceFalloffWeight));
		if(ghostViewBasedOnClipping == 1) {
			alpha = max(alpha, texture2D(contoursMap, gl_TexCoord[0].st).r * clippingWeight);
		}
	}

	alpha = clamp(alpha, 0.0, 1.0);

	if(ARPolygonal)
		fragColor = computeFragmentColorARPolygonal(realDepth, virtualDepth, realRGB, virtualRGB, alpha, threshold);
	else if(ARFromKinectFusionVolume)
		fragColor = computeFragmentColorARFromKinectFusionVolume(realDepth, virtualDepth, realRGB, virtualRGB, alpha, threshold);
	else {
		if(ghostViewBasedOnSubtractionMaskCase1 == 0 && ghostViewBasedOnSubtractionMaskCase2 == 0)
			fragColor = computeFragmentColorARFromVolumeRendering(realDepth, virtualDepth, realRGB, virtualRGB, alpha, threshold);
		else if(ghostViewBasedOnSubtractionMaskCase1 == 1) {
			vec4 backgroundRGB = texture2D(backgroundMap, gl_TexCoord[0].st);
			vec4 subtractionRGB = texture2D(subtractionMap, gl_TexCoord[0].st);
			//threshold = 0.025;
			threshold = 0.05;
			fragColor = computeFragmentColorARFromVolumeRendering(realDepth, virtualDepth, realRGB, virtualRGB, alpha, threshold, backgroundRGB, subtractionRGB, 
				grayLevelWeight);
		} else {
			vec4 backgroundRGB = texture2D(backgroundMap, gl_TexCoord[0].st);
			vec4 subtractionRGB = texture2D(subtractionMap, gl_TexCoord[0].st);
			//threshold = 0.025;
			threshold = 0.05;
			fragColor = computeFragmentColorARFromVolumeRendering(realDepth, virtualDepth, realRGB, virtualRGB, alpha, threshold, backgroundRGB, subtractionRGB);
		}
	}

	gl_FragColor = fragColor;

}