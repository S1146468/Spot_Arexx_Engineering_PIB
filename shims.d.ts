// Auto-generated. Do not edit.



    //%
declare namespace Spot {

    /**
     * Inizialize Spot
     */
    //% shim=Spot::Init_Spot_cpp
    function Init_Spot_cpp(): void;

    /**
     * Demo function
     */
    //% shim=Spot::Demo_cpp
    function Demo_cpp(): void;

    /**
     * C++ function for walking
     * Speed unit is in %
     * Distance unit is in cm
     */
    //% shim=Spot::Walk_cpp
    function Walk_cpp(direction: directions, speed: int32, distance: int32): void;

    /**
     * C++ function for turning
     * Speed unit is in %
     * Angle unit is in degrees
     */
    //% shim=Spot::Turn_cpp
    function Turn_cpp(direction: directions, speed: int32, angle: int32): void;
}

// Auto-generated. Do not edit. Really.
