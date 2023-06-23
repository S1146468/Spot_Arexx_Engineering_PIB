/**
* Spot Arexx Engineering
*/
//% color=#000000 weight=1000 icon="\uf04b" block="Spot Arexx Engineering"
//% groups='["Startup", "Walking", "Turning"]'
namespace Spot {
    /**
    * Place this in Start block
    */
    //% group="Startup"
    //% block="Start Spot"
    //% weight=20
    export function Init_Spot() {
        Init_Spot_cpp();
    }

    /**
     * Demo function
     */
    //% group="Startup"
    //% block="Demo"
    //% weight=10
    export function Demo() {
        Demo_cpp();
    }

    /**
     * Walk forward
     */
    //% group="Walking"
    //% block="Walk forward"
    //% weight=80
    export function Walk_Forward() {
        Walk(directions.forward, 5, 6);
    }

    /**
    * Walk backward
    */
    //% group="Walking"
    //% block="Walk backward"
    //% weight=70
    export function Walk_Backward() {
        Walk(directions.backward, 5, 6);
    }

    /**
    * Walk to the left
    */
    //% group="Walking"
    //% block="Walk to the left"
    //% weight=60
    export function Walk_Left() {
        Walk(directions.left, 5, 6);
    }

    /**
    * Walk to the right
    */
    //% group="Walking"
    //% block="Walk to the right"
    //% weight=50
    export function Walk_Right() {
        Walk(directions.right, 5, 6);
    }

    /**
    * Turn around
    */
    //% group="Turning"
    //% block="Turn around"
    //% weight=40
    export function Turn_Arount() {
        Turn_cpp(directions.left, 5, 180);
    }

    /**
    * Turn a bit to the left
    */
    //% group="Turning"
    //% block="Turn left"
    //% weight=30
    export function Turn_Left() {
        Turn_cpp(directions.left, 5, 90);
    }

    /**
    * Turn a bit to the right
    */
    //% group="Turning"
    //% block="Turn right"
    //% weight=20
    export function Turn_Right() {
        Turn_cpp(directions.right, 5, 90);
    }

    /**
    * Walk for a defined distance with defined speed and direction
    * @param distance The distance is going to walk in cm, eg: 5
    * @param direction direction of walking, eg: forward
    * @param speed The speed Spot going to walk at, eg: 5
    */
    //% speed.min=1 speed.max=10
    //% distance.min=1 distance.max=100
    //% group="Walking"
    //% weight=30
    //% block="Walk %distance cm %direction with a speed of %speed"
    //% advanced=true
    export function Walk(direction: directions, speed: number, distance: number): void {
        Walk_cpp(direction, speed, distance);
    }

    /**
    * Turn left or right for a defined angle with a defined speed
    * @param angle The distance is going to walk in cm, eg: 5
    * @param direction direction of walking, eg: forward
    * @param speed The speed Spot going to walk at, eg: 5
    */
    //% speed.min=1 speed.max=10
    //% angle.min=1 angle.max=360
    //% group="Walking"
    //% weight=30
    //% block="Turn %angle degrees %direction with a speed of %speed"
    //% advanced=true
    export function Turn(direction: directions, speed: number, angle: number): void {
        Turn_cpp(direction, speed, angle);
    }
}