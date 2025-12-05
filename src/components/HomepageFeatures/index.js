import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'Embodied Intelligence',
    // Yahan humne 'Svg' ki jagah 'image' key use ki hai kyunki ye PNG hai
    // .default lagana zaroori hai taake sahi path mile
    image: require('@site/static/img/embodied.png').default,
    description: (
      <>
        Move beyond digital screens. Bridging the crucial gap between advanced AI models ("digital brains") and physical humanoid bodies operating in the real world.
      </>
    ),
  },
  {
    title: 'Master the Robotic Stack',
    // Ye SVG hai, isliye isay 'Svg' key mein hi rakhenge
  image: require('@site/static/img/robots2.png').default,
    description: (
      <>
        Build the robotic "nervous system" using industry-standard <code>ROS 2</code> (Python/rclpy). Create high-fidelity digital twins in Gazebo and Unity before real-world deployment.
      </>
    ),
  },
  {
    title: 'Next-Gen AI Brains',
    // Ye bhi SVG hai
    image: require('@site/static/img/brain1.png').default,
    description: (
      <>
        Implement advanced perception and cognitive control using cutting-edge tools like <code>NVIDIA Isaac Sim</code>, OpenAI Whisper, and Vision-Language-Action (VLA) models.
      </>
    ),
  },
];

// Feature function ko update kiya hai taake wo check kare ke Image hai ya SVG
function Feature({Svg, image, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        {/* Logic: Agar Svg hai toh Component banao, Agar image hai toh img tag lagao */}
        {Svg ? (
          <Svg className={styles.featureSvg} role="img" />
        ) : (
          <img src={image} className={styles.featureSvg} alt={title} />
        )}
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}