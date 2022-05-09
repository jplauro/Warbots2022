package frc.robot.Util;

import java.util.Map;
import java.util.TreeMap;

public class LinearInterpolationMap {
    private TreeMap<Double, Double> map = new TreeMap<>();

    /** 
     * Determines whether the specified key is between the 
     * lowest and highest keys in a non-empty tree map
     * 
     * @param key The key to check against
     * @return    Whether the key is inside the bounds
     */
    public boolean isKeyInBounds(Double key) {
        return !this.map.isEmpty() && key >= this.map.firstKey() && key <= this.map.lastKey();
    }

    /** 
     * Inserts a key-value pair into the map
     * 
     * @param key   The input value used for interpolation
     * @param value The output value that is interpolated from
     */
    public void put(Double key, Double value) {
        this.map.put(key, value);
    }

    /** 
     * Inserts a group of key-value pairs into the map
     * 
     * @param map The group of values used for interpolation
     */
    public void putAll(Map<Double, Double> map) {
        this.map.putAll(map);
    }

    /** 
     * Calculates the interpolated value of the specified 
     * key and returns the closest value if out of 
     * bounds or null if the tree map is empty
     * 
     * @param key The key to interpolate from
     * @return    The interpolated value from the key
     */
    public Double get(Double key) {
        return this.get(key, null);
    }

    /** 
     * Calculates the interpolated value of the specified
     * key and returns the default value if out of bounds
     * 
     * @param key          The key to interpolate from
     * @param defaultValue The value to return if out of bounds
     * @return             The interpolated value from the key
     */
    public Double get(Double key, Double defaultValue) {
        // Return the default value if the specified key is out of bounds
        if (defaultValue != null && !this.isKeyInBounds(key))
            return defaultValue;
        // Return the associated value if a matching key happens to exist
        else if (this.map.containsKey(key))
            return this.map.get(key);
        else {
            // Get upper and lower keys for interpolation
            Double lowerKey = this.map.floorKey(key);
            Double upperKey = this.map.ceilingKey(key);

            // Return the nearest data point if at tree edge
            if (lowerKey == null && upperKey == null) {
                // Return null if tree is empty
                return null;
            } else if (lowerKey == null) {
                // Return lowest value if key is below the minimum limit
                return this.map.get(upperKey);
            } else if (upperKey == null)
                // Return highest value if key is above the maximum limit
                return this.map.get(lowerKey);

            // Get the various values for interpolation
            Double lowerValue = this.map.get(lowerKey);
            Double keyToLower = key - lowerKey;
            Double upperToLower = upperKey - lowerKey;

            // Verify that undefined and unwanted values are not produced in division
            Double inverse = keyToLower <= 0 || upperToLower <= 0 ? 0 : keyToLower / upperToLower;
            return (this.map.get(upperKey) - lowerValue) * inverse + lowerValue;
        }
    }
}
