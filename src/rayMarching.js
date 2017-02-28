const THREE = require('three');
const EffectComposer = require('three-effectcomposer')(THREE)

import {PROXY_BUFFER_SIZE} from './proxy_geometry'

var options = {
    time: 0
}

export default function RayMarcher(renderer, scene, camera) {
    var composer = new EffectComposer(renderer);
    var shaderPass = new EffectComposer.ShaderPass({
        uniforms: {
            u_buffer: {
                type: '4fv',
                value: undefined
            },
            u_count: {
                type: 'i',
                value: 0
            },
            cameraPos: {
                type: '3fv',
                value: camera.position
            },
            cameraTransform:
            {
                type: 'm4',
                value: camera.matrix
            },
            tanAlpha:
            {
                type: 'f',
                value: Math.tan(camera.fov/2.0 * Math.PI/180.0)
            },
            aspRatio:
            {
                type: 'f',
                value: window.innerWidth/window.innerHeight
            },
            time:
            {
                type: 'i',
                value: options.time
            },
        },
        vertexShader: require('./glsl/pass-vert.glsl'),
        fragmentShader: require('./glsl/rayMarch-frag.glsl')
    });
    shaderPass.renderToScreen = true;
    composer.addPass(shaderPass);

    return {
        render: function(buffer) {
            shaderPass.material.uniforms.u_buffer.value = buffer;
            shaderPass.material.uniforms.u_count.value = buffer.length / PROXY_BUFFER_SIZE;

            camera.updateMatrixWorld();

            // var tempF = new THREE.Vector4(0.0, 0.0, -1.0, 0.0);
            // var temp = tempF.applyMatrix4(camera.matrix);
            // var F = new THREE.Vector3(temp[0], temp[1], temp[2]);  console.log(F);F.normalize();
            // var R = (F.cross(THREE.Vector3(0.0, 1.0, 0.0))).normalize();
            // var U = R.cross(F).normalize();
            // var ref = camera.position + 0.1*F;
            // var len = THREE.Vector3(ref - cameraPos).length();
            // var tanAlpha = Math.tan(camera.fov/2.0 * Math.PI/180.0);
            // var p = ref + ((2.0*f_uv.x-1.0) * (R*len*camera.aspect*tanAlpha)) + ((2.0*f_uv.y-1.0) * (U*len*tanAlpha));

            shaderPass.material.uniforms.cameraTransform.value = camera.matrix;
            shaderPass.material.uniforms.cameraPos.value = camera.position;
            options.time++;
            shaderPass.material.uniforms.time.value = options.time;

            composer.render();
        },

        update: function() {
            
        }
    }
}