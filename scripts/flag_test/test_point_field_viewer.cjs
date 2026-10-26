const assert = require("node:assert/strict");
const {test} = require("node:test");
const {visiblePoints} = require("./point_field_viewer.js");

test("a narrow view uses every original point at its index", () => {
  const values = new Float64Array(20000).fill(5);
  values[1002] = NaN;
  const result = visiblePoints(values, 1000, 1004);
  assert.deepEqual(result.x, [1000, 1001, 1002, 1003, 1004]);
  assert.deepEqual(result.y, [5, 5, null, 5, 5]);
});

test("overview is bounded and retains isolated minimum and maximum", () => {
  const values = new Float64Array(100000).fill(5);
  values[1234] = -9;
  values[45678] = 99;
  const result = visiblePoints(values, 0, values.length - 1);
  assert.ok(result.x.length <= 6000);
  assert.equal(result.x[0], 0);
  assert.equal(result.x.at(-1), values.length - 1);
  assert.ok(result.x.includes(1234));
  assert.ok(result.x.includes(45678));
  assert.ok(result.x.every((index, position) => position === 0 || result.x[position - 1] <= index));
});
