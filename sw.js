// Service Worker fuer die Messgeraet-PWA.
// Cache-first fuer die App-Huelle, damit die Seite nach dem ersten
// Aufruf auch komplett ohne Internetverbindung startet. Die Bluetooth-
// Verbindung zum ESP32 selbst laeuft ohnehin unabhaengig davon.

const CACHE_NAME = "messgeraet-cache-v4";
const ASSETS = [
  "./",
  "./index.html",
  "./manifest.json",
  "./icons/icon-192.png",
  "./icons/icon-512.png"
];

self.addEventListener("install", event => {
  event.waitUntil(
    caches.open(CACHE_NAME)
      .then(cache => cache.addAll(ASSETS))
      .catch(err => console.log("SW install: konnte nicht alles cachen:", err))
  );
  self.skipWaiting();
});

self.addEventListener("activate", event => {
  event.waitUntil(
    caches.keys().then(keys =>
      Promise.all(keys.filter(k => k !== CACHE_NAME).map(k => caches.delete(k)))
    )
  );
  self.clients.claim();
});

self.addEventListener("fetch", event => {
  if (event.request.method !== "GET") return;

  event.respondWith(
    caches.match(event.request).then(cached => {
      const network = fetch(event.request)
        .then(response => {
          if (response && response.status === 200) {
            const clone = response.clone();
            caches.open(CACHE_NAME).then(cache => cache.put(event.request, clone));
          }
          return response;
        })
        .catch(() => cached);   // offline: auf den Cache zurueckfallen

      // Sofort aus dem Cache liefern, falls vorhanden (schnell + offline-fest),
      // im Hintergrund trotzdem aktualisieren, wenn Internet da ist.
      return cached || network;
    })
  );
});
