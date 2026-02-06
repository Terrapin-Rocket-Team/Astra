#ifndef MOUNTING_TRANSFORM_H
#define MOUNTING_TRANSFORM_H

#include "../Math/Vector.h"

namespace astra
{
    /**
     * MountingOrientation - Common mounting orientations for sensors
     *
     * These represent 90-degree rotations commonly encountered when mounting
     * sensors on a PCB that may be oriented differently from the rocket body.
     *
     * The naming convention is: AXIS_DIRECTION where AXIS is the sensor's
     * positive X-axis direction in the body frame, and DIRECTION is which
     * way the sensor's positive Z-axis points.
     */
    enum class MountingOrientation
    {
        // Identity - sensor frame = body frame
        // Sensor X → Body X (forward), Sensor Y → Body Y (right), Sensor Z → Body Z (down)
        IDENTITY,

        // 180° rotation about X axis
        // Sensor X → Body X, Sensor Y → Body -Y, Sensor Z → Body -Z
        FLIP_YZ,

        // 180° rotation about Y axis
        // Sensor X → Body -X, Sensor Y → Body Y, Sensor Z → Body -Z
        FLIP_XZ,

        // 180° rotation about Z axis
        // Sensor X → Body -X, Sensor Y → Body -Y, Sensor Z → Body Z
        FLIP_XY,

        // 90° rotation about X axis (Y→Z, Z→-Y)
        ROTATE_90_X,

        // -90° rotation about X axis (Y→-Z, Z→Y)
        ROTATE_NEG90_X,

        // 90° rotation about Y axis (X→-Z, Z→X)
        ROTATE_90_Y,

        // -90° rotation about Y axis (X→Z, Z→-X)
        ROTATE_NEG90_Y,

        // 90° rotation about Z axis (X→Y, Y→-X)
        ROTATE_90_Z,

        // -90° rotation about Z axis (X→-Y, Y→X)
        ROTATE_NEG90_Z,

        // Custom - use the provided matrix
        CUSTOM
    };

    /**
     * MountingTransform - Transforms vectors from sensor frame to body frame
     *
     * Handles the static rotation between a sensor's native coordinate system
     * and the rocket's body frame. This is a fixed transformation that depends
     * on how the sensor is physically mounted.
     *
     * Usage:
     *   MountingTransform mount(MountingOrientation::FLIP_YZ);  // Upside-down sensor
     *   Vector<3> bodyAccel = mount.transform(sensorAccel);
     */
    class MountingTransform
    {
    public:
        /**
         * Create a mounting transform from a preset orientation
         */
        MountingTransform(MountingOrientation orientation = MountingOrientation::IDENTITY)
            : orientation(orientation)
        {
            setOrientation(orientation);
        }

        /**
         * Create a mounting transform from axis signs
         * Each parameter is +1 or -1 indicating if that axis is flipped
         * This only handles axis flips, not rotations
         */
        MountingTransform(int xSign, int ySign, int zSign)
            : orientation(MountingOrientation::CUSTOM)
        {
            // Simple axis flip/sign matrix
            matrix[0] = xSign; matrix[1] = 0;     matrix[2] = 0;
            matrix[3] = 0;     matrix[4] = ySign; matrix[5] = 0;
            matrix[6] = 0;     matrix[7] = 0;     matrix[8] = zSign;
        }

        /**
         * Create a mounting transform from a full 3x3 rotation matrix
         * Matrix is row-major: [r00, r01, r02, r10, r11, r12, r20, r21, r22]
         */
        MountingTransform(const double* rotationMatrix)
            : orientation(MountingOrientation::CUSTOM)
        {
            for (int i = 0; i < 9; i++)
                matrix[i] = rotationMatrix[i];
        }

        /**
         * Set the orientation using a preset
         */
        void setOrientation(MountingOrientation orient)
        {
            orientation = orient;

            // Initialize to identity
            matrix[0] = 1; matrix[1] = 0; matrix[2] = 0;
            matrix[3] = 0; matrix[4] = 1; matrix[5] = 0;
            matrix[6] = 0; matrix[7] = 0; matrix[8] = 1;

            switch (orient)
            {
            case MountingOrientation::IDENTITY:
                // Already identity
                break;

            case MountingOrientation::FLIP_YZ:
                // 180° about X: Y→-Y, Z→-Z
                matrix[4] = -1; matrix[8] = -1;
                break;

            case MountingOrientation::FLIP_XZ:
                // 180° about Y: X→-X, Z→-Z
                matrix[0] = -1; matrix[8] = -1;
                break;

            case MountingOrientation::FLIP_XY:
                // 180° about Z: X→-X, Y→-Y
                matrix[0] = -1; matrix[4] = -1;
                break;

            case MountingOrientation::ROTATE_90_X:
                // 90° about X: Y→Z, Z→-Y
                matrix[4] = 0; matrix[5] = 1;
                matrix[7] = -1; matrix[8] = 0;
                break;

            case MountingOrientation::ROTATE_NEG90_X:
                // -90° about X: Y→-Z, Z→Y
                matrix[4] = 0; matrix[5] = -1;
                matrix[7] = 1; matrix[8] = 0;
                break;

            case MountingOrientation::ROTATE_90_Y:
                // 90° about Y: X→-Z, Z→X
                matrix[0] = 0; matrix[2] = -1;
                matrix[6] = 1; matrix[8] = 0;
                break;

            case MountingOrientation::ROTATE_NEG90_Y:
                // -90° about Y: X→Z, Z→-X
                matrix[0] = 0; matrix[2] = 1;
                matrix[6] = -1; matrix[8] = 0;
                break;

            case MountingOrientation::ROTATE_90_Z:
                // 90° about Z: X→Y, Y→-X
                matrix[0] = 0; matrix[1] = 1;
                matrix[3] = -1; matrix[4] = 0;
                break;

            case MountingOrientation::ROTATE_NEG90_Z:
                // -90° about Z: X→-Y, Y→X
                matrix[0] = 0; matrix[1] = -1;
                matrix[3] = 1; matrix[4] = 0;
                break;

            case MountingOrientation::CUSTOM:
                // Leave as identity - caller should set matrix directly
                break;
            }
        }

        /**
         * Set a custom rotation matrix
         * Matrix is row-major: [r00, r01, r02, r10, r11, r12, r20, r21, r22]
         */
        void setMatrix(const double* rotationMatrix)
        {
            orientation = MountingOrientation::CUSTOM;
            for (int i = 0; i < 9; i++)
                matrix[i] = rotationMatrix[i];
        }

        /**
         * Transform a vector from sensor frame to board frame
         */
        Vector<3> transform(const Vector<3>& sensorVec) const
        {
            return Vector<3>(
                matrix[0] * sensorVec.x() + matrix[1] * sensorVec.y() + matrix[2] * sensorVec.z(),
                matrix[3] * sensorVec.x() + matrix[4] * sensorVec.y() + matrix[5] * sensorVec.z(),
                matrix[6] * sensorVec.x() + matrix[7] * sensorVec.y() + matrix[8] * sensorVec.z()
            );
        }

        /**
         * Get the current orientation preset (CUSTOM if using custom matrix)
         */
        MountingOrientation getOrientation() const { return orientation; }

        /**
         * Get read-only access to the rotation matrix
         */
        const double* getMatrix() const { return matrix; }

        /**
         * Check if this is an identity transform (no rotation needed)
         */
        bool isIdentity() const
        {
            return orientation == MountingOrientation::IDENTITY ||
                   (matrix[0] == 1 && matrix[4] == 1 && matrix[8] == 1 &&
                    matrix[1] == 0 && matrix[2] == 0 && matrix[3] == 0 &&
                    matrix[5] == 0 && matrix[6] == 0 && matrix[7] == 0);
        }

    private:
        MountingOrientation orientation;
        double matrix[9];  // 3x3 row-major rotation matrix
    };

} // namespace astra

#endif // MOUNTING_TRANSFORM_H
