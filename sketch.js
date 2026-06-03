// Red clip styling for GuiController peak meters (dat.gui rows).
var sketch = function(p) {
  p.setup = function() {
    p.noCanvas();
  };

  p.draw = function() {
    applyClipMeterStyles();
  };
};

new p5(sketch);

function applyClipMeterStyles() {
  var rows = document.querySelectorAll('.dg .cr');
  for (var i = 0; i < rows.length; i++) {
    var row = rows[i];
    var label = row.querySelector('.property-name');
    if (!label || label.textContent.indexOf('Peak (1=clip)') === -1)
      continue;

    var input = row.querySelector('input[type="text"]');
    var val = input ? parseFloat(input.value) : 0;
    var clip = val >= 1.0;

    row.classList.toggle('clip-warning', clip);
    label.style.color = clip ? '#e33' : '';
    label.style.fontWeight = clip ? '600' : '';
    if (input) input.style.color = clip ? '#e33' : '';
  }
}
