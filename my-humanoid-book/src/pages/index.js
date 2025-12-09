
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={styles.heroBanner}>
      <div className="container">
        <h1 className={styles.heroTitle}>
          Humanoid Robotics & Physical AI
        </h1>

        <p className={styles.heroSubtitle}>
          A modern, practical, and project-based <strong>step-by-step</strong> guide to building AI-powered humanoid robots.
        </p>

        <div className={styles.buttons}>
          <Link className="button button--primary button--lg" to="/docs/intro">
            Get Started 🚀
          </Link>

          <button
            className="button button--secondary button--lg"
            onClick={() => {
              document.getElementById('modules').scrollIntoView({ behavior: 'smooth' });
            }}
          >
            Explore Modules
          </button>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`Home | ${siteConfig.title}`}
      description="Humanoid Robotics & Physical AI — A modern guide to building intelligent humanoid systems."
    >
      <HomepageHeader />
      <main>
        <HomepageFeatures id="modules" />
      </main>
    </Layout>
  );
}
