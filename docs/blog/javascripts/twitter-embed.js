(() => {
  const STATUS_URL_RE = /^https?:\/\/(?:mobile\.)?(?:twitter\.com|x\.com)\/([^/]+)\/status\/(\d+)/i;

  function parseStatusUrl(url) {
    try {
      const u = new URL(url);
      const match = `${u.origin}${u.pathname}`.match(STATUS_URL_RE);
      if (!match) return null;
      return {
        canonicalUrl: `https://twitter.com/${match[1]}/status/${match[2]}`,
      };
    } catch (error) {
      return null;
    }
  }

  function convertBlockquotesToTweetEmbeds() {
    let converted = 0;
    const blockquotes = document.querySelectorAll("blockquote");

    blockquotes.forEach((blockquote) => {
      if (blockquote.classList.contains("twitter-tweet")) return;

      const links = blockquote.querySelectorAll("a[href]");
      let tweet = null;

      for (const link of links) {
        const parsed = parseStatusUrl(link.href);
        if (parsed) {
          tweet = parsed;
          break;
        }
      }

      if (!tweet) return;

      const embed = document.createElement("blockquote");
      embed.className = "twitter-tweet";
      embed.setAttribute("data-conversation", "none");
      embed.setAttribute("data-dnt", "true");

      const anchor = document.createElement("a");
      anchor.href = tweet.canonicalUrl;
      anchor.textContent = tweet.canonicalUrl;
      embed.appendChild(anchor);

      blockquote.replaceWith(embed);
      converted += 1;
    });

    return converted;
  }

  function ensureTwitterWidgets() {
    if (
      window.twttr &&
      window.twttr.widgets &&
      typeof window.twttr.widgets.load === "function"
    ) {
      window.twttr.widgets.load();
      return;
    }

    if (document.querySelector('script[data-twitter-widgets="1"]')) {
      return;
    }

    const script = document.createElement("script");
    script.src = "https://platform.twitter.com/widgets.js";
    script.async = true;
    script.charset = "utf-8";
    script.onload = function () {
      if (
        window.twttr &&
        window.twttr.widgets &&
        typeof window.twttr.widgets.load === "function"
      ) {
        window.twttr.widgets.load();
      }
    };
    script.setAttribute("data-twitter-widgets", "1");
    document.head.appendChild(script);
  }

  function init() {
    const converted = convertBlockquotesToTweetEmbeds();
    if (converted > 0) {
      ensureTwitterWidgets();
    }
  }

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", init, { once: true });
  } else {
    init();
  }
})();
