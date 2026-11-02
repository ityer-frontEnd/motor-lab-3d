// Build a fully self-contained single-file deliverable.
// Three.js r187 is bundled to one ES module and embedded as a data: URL in the import map,
// so index.html runs offline from file:// without any local server or remote dependency.
import fs from 'node:fs';
import path from 'node:path';

const root = path.resolve(import.meta.dirname, '..');
const threePath = path.join(root, 'dist', 'three.bundle.js');
const templatePath = path.join(root, 'src', 'index.template.html');
const appPath = path.join(root, 'src', 'app.js');
const outPath = path.join(root, 'index.html');

for (const p of [threePath, templatePath, appPath]) {
  if (!fs.existsSync(p)) {
    console.error(`Missing build input: ${p}`);
    process.exit(1);
  }
}

const three = fs.readFileSync(threePath);
const b64 = three.toString('base64');
const importMap = JSON.stringify({
  imports: {
    three: `data:text/javascript;base64,${b64}`
  }
});

const app = fs.readFileSync(appPath, 'utf8').replace(/<\/script/gi, '<\\/script');
let html = fs.readFileSync(templatePath, 'utf8');

if (!html.includes('__THREE_IMPORTMAP__') || !html.includes('__APP_JS__')) {
  console.error('Template placeholders are missing.');
  process.exit(1);
}
html = html.replace('__THREE_IMPORTMAP__', () => importMap);
html = html.replace('__APP_JS__', () => app);

fs.writeFileSync(outPath, html, 'utf8');
const size = fs.statSync(outPath).size;
console.log(`Built ${outPath} (${(size / 1024 / 1024).toFixed(2)} MB) — Three.js r187 pinned via import map.`);
