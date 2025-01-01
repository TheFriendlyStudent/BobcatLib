

function updateAll() {
  let jsons = ['elevator'];
  jsons.forEach((json) => {
    updateJSON(json);
  });
}

function updateJSON(name) {
  let form = $(`#${name}-json`);
  if (form.text() !== jsonify(name)) {
    form.text(jsonify(name));
  }
  // $(`#${name}-test`).text(jsonify(name));
}

function jsonify(name) {
  let data = {};
  let formdata = $(`#${name}-form`).serializeArray();
  formdata.forEach((val) => {
    if (val.name.includes('_')) {
      let struct = val.name.substring(0, val.name.indexOf('_'));
      if (data[struct] === undefined) {
        data[struct] = {};
      }
      data[struct][val.name.substring(val.name.indexOf('_') + 1)] =
        isNumeric(val.value) ?
          parseFloat(val.value) :
          (val.value === "" ? null : val.value);
    } else {
      data[val.name] = isNumeric(val.value) ?
        parseFloat(val.value) :
        (val.value === "" ? null : val.value);
    }
  });
  $(`#${name}-form input:checkbox`).each(function () { // Update checkboxes because unchecked checkboxes are not recognized by jQuery serializeArray
    if (this.name.includes('_')) {
      let struct = this.name.substring(0, this.name.indexOf('_'));
      if (data[struct] === undefined) {
        data[struct] = {};
      }
      data[struct][this.name.substring(
        this.name.indexOf('_') + 1)] = this.checked;
    } else {
      data[this.name] = this.checked;
    }
  });
  return JSON.stringify(data, null, 2);
}

function isNumeric(str) {
  if (typeof str != "string") {
    return false
  } // we only process strings!
  return !isNaN(str) && // use type coercion to parse the _entirety_ of the string (`parseFloat` alone does not do this)...
    !isNaN(parseFloat(str)) // ...and ensure strings of whitespace fail
}

function copyText(name) {
  let text = $(`#${name}-json`).text();
  navigator.clipboard.writeText(text);
}

function getText(name) {
  let text = $(`#${name}-json`).text();
  return text;
}

//function from https://github.com/eligrey/FileSaver.js/issues/774
const saveAs = (blob, name) => {
  // Namespace is used to prevent conflict w/ Chrome Poper Blocker extension (Issue https://github.com/eligrey/FileSaver.js/issues/561)
  const a = document.createElementNS('http://www.w3.org/1999/xhtml', 'a')
  a.download = name
  a.rel = 'noopener'
  a.href = URL.createObjectURL(blob)

  setTimeout(() => URL.revokeObjectURL(a.href), 40 /* sec */ * 1000)
  setTimeout(() => a.click(), 0)
}

function deploy() {
  let jsons = ['elevator'];
  jsons.forEach((json) => {
    downloadText(json);
  });
}

function uploadText(name) {
  let body = getText(name)
  $.post("http://localhost:5004/api/deploy/" + name, body, (data, status) => {
    console.log(data);
  });
}
function downloadText(name) {
  let text = $(`#${name}-json`).text();
  const zip = new JSZip();
  let swf = zip.folder("Elevator")
  let filename = name + ".json"
  console.log(filename)
  let imu = swf.file(filename, getText(name));

  zip.generateAsync({ type: "blob" }).then(function (blob) { saveAs(blob, `Elevator ${name} Config.zip`) });
  console.log("Downloaded " + name + " config");
}

function zipDownload() {
  const zip = new JSZip();
  let swf = zip.folder("Elevator")
  let elevator = swf.file("elevator.json", getText("elevator"));

  zip.generateAsync({ type: "blob" }).then(function (blob) { saveAs(blob, "Elevator Config.zip") });
  console.log("Downloaded Basic Swerve Config zip");
}

$(function () {
  const tooltipTriggerList = document.querySelectorAll(
    '[data-bs-toggle="tooltip"]'); // Initialize tooltips: https://getbootstrap.com/docs/5.3/components/tooltips/#enable-tooltips
  const tooltipList = [...tooltipTriggerList].map(
    tooltipTriggerEl => new bootstrap.Tooltip(tooltipTriggerEl));

  $('a').click(function () {
    window.open(this.href, '_blank');
    return false;
  });

  updateAll();
  setInterval(updateAll, 500);
});