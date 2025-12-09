
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

const FeatureList = [
  {
    module: 'Module 1',
    title: 'The Robotic Nervous System (ROS 2)',
    description: "Build the robot's control & communication system with ROS 2.",
    link: '/docs/intro',
  },
  {
    module: 'Module 2',
    title: 'The Digital Twin (Gazebo & Unity)',
    description: 'Create a digital twin of a humanoid robot for safe simulation and testing.',
    link: '/docs/intro',
  },
  {
    module: 'Module 3',
    title: 'The AI Brain (NVIDIA Isaac)',
    description: 'Use NVIDIA Isaac for SLAM, navigation, and robot perception.',
    link: '/docs/intro',
  },
  {
    module: 'Module 4',
    title: 'Vision-Language-Action Robotics (VLA)',
    description: 'Control robots via natural language with VLA.',
    link: '/docs/intro',
  },
];

function Feature({ module, title, description, link }) {
  return (
    <div className={clsx(styles.featureCard)}>
      <div className={styles.cardContent}>
        <h4 className={styles.moduleNumber}>{module}</h4>
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
        <a href={link} className={styles.readButton}>Read Module</a>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section id="modules" className={styles.features}>
      <div className={styles.cardsWrapper}>
        {FeatureList.map((props, idx) => (
          <Feature key={idx} {...props} />
        ))}
      </div>
    </section>
  );
}
